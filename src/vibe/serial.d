﻿/**
 * 
 */
module vibe.serial;
// Qualifier order in this module: final [void foo()] @property const @safe pure nothrow @nogc

import core.stdc.errno, core.sys.posix.unistd, core.sys.posix.fcntl, core.sys.posix.termios;
import std.exception;
import vibe.d;

/**
 * Support mocking low level posix functions for testing
 */
version(MockTest)
{
    void delegate() posix_open_del;
    void posix_open()
    {
        return posix_open_del();
    }
}
else
{
    private alias posix_open = core.sys.posix.fcntl.open;
    private alias posix_close = core.sys.posix.unistd.close;
    private alias posix_read = core.sys.posix.unistd.read;
    private alias posix_write = core.sys.posix.unistd.write;
    private alias posix_tcgetattr = core.sys.posix.termios.tcgetattr;
    private alias posix_tcsetattr = core.sys.posix.termios.tcsetattr;
}

/**
 * 
 */
enum BaudRate
{
    B50,
    B75,
    B110,
    B134,
    B150,
    B200,
    B300,
    B600,
    B1200,
    B1800,
    B2400,
    B4800,
    B9600,
    B19200,
    B38400,
    B57600,
    B76800,
    B115200
}

/**
 * 
 */
enum DataBits
{
    five,
    six,
    seven,
    eight
}

/**
 * 
 */
enum Parity
{
    none,
    odd,
    even
}

/**
 * 
 */
enum StopBits
{
    one,
    two
}

enum FlowControl
{
    none,
    software,
    hardware
}

/**
 * Failed to set a parameter of the serial port.
 */
class ParameterException : Exception
{
private:
    this(ParameterException.Type type, string file =__FILE__, size_t line = __LINE__, Throwable next = null) @safe pure nothrow 
    {
        parameter = type;
        string paramString;
        try
            paramString = to!string(type);
        catch(Exception){}
        super("Couldn't set a serial port parameter: " ~ paramString, file, line, next);
    }

public:
    /**
     * Possible parameters to configure serial port.
     */
    enum Type
    {
        internal,
        baudRate,
        parity,
        stopBits,
        dataBits,
        flowControl
    }

    /**
     * Which parameter couldn't be set
     */
    Type parameter;
}

// TODO: check whether interrupting works

/**
 * 
 */
class TimeoutException
{

    size_t bytesProcessed;
}

/**
 * TODO: Should this automatically close the fd
 * on flush, write, read Exceptions?
 */
class SerialConnection : ConnectionStream
{
private:
    FileDescriptorEvent _readEvent, _writeEvent;
    int _fd = -1;

    ubyte[2048] _buffer;
    ubyte[] _bufAvailable;
    bool _eof = false;

    /// Get serial port options
    termios getOptions() const @trusted
    {
        termios options;
        errnoEnforce(posix_tcgetattr(_fd, &options) != -1, "Couldn't get terminal options");
        return options;
    }
    
    /**
     * Set serial port options.
     *
     * Returns: True if set sucessfully, false if invalid input value
     * Throws: ErrnoException (except for invalid input value)
     */
    bool setOptions(in termios options) @trusted
    {
        auto result = posix_tcsetattr(_fd, TCSADRAIN, &options);
        if (result == -1)
        {
            if (errno == EINVAL)
                return false;
            else
                errnoEnforce(false, "Couldn't set terminal options");
        }
        else
            return true;

        // Can't happen
        assert(0);
    }

    bool setVerifyOptions(in termios value) @safe
    {
        if(setOptions(value))
        {
            auto newOptions = getOptions();
            return newOptions == value;
        }
        else
            return false;
    }

    void setBaseParams() @safe
    {
        auto options = getOptions();
        // Start receiver
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= ~(HUPCL);
        
        // Raw input
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHOK | ECHONL | IEXTEN | TOSTOP);
        
        // No postprocessing
        options.c_oflag &= ~(OPOST | /*OLCUC |*/ ONLCR | OCRNL | ONOCR | ONLRET | OFILL);
        
        // Ignore break condition
        options.c_iflag &= ~(IXON | IXOFF | IXANY | BRKINT | ICRNL | INLCR | ISTRIP | IGNCR /*| IUCLC*/);
        options.c_iflag |= IGNBRK;

        if(!setVerifyOptions(options))
            throw new ParameterException(ParameterException.Type.internal);
    }

    /**
     * Read into buffer. Return number of bytes read.
     */
    size_t readInto(scope ubyte[] buf) @trusted
    {
        assert(buf.length > 0);

        auto res = posix_read(_fd, buf.ptr, buf.length);

        if (res == -1)
        {
            // FIXME: Should this handle EINTR as well? Also check EINTR for all other syscalls
            // Return 0, try again
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return 0;
            else
                errnoEnforce(false, "Failed to read data");
        }
        else if (res == 0)
        {
            // End of File
            _eof = true;
            return 0;
        }
        else
            return res;

        // Unreachable
        assert(0);
    }

    /**
     * Write from buffer. Return bytes written.
     */
    size_t writeFrom(const(ubyte)[] buf) @trusted
    {
        assert(buf.length > 0);

        auto res = posix_write(_fd, buf.ptr, buf.length);
        
        if (res == -1)
        {
            // FIXME: Should this handle EINTR as well? Also check EINTR for all other syscalls
            // Return 0, try again
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return 0;
            else
                errnoEnforce(false, "Failed to write data");
        }
        else
            return res;

        return 0;
    }

    /**
     * Refill internal buffer. After a call to this function, _bufAvailable.length
     * is greater than zero, or _eof is set to true.
     */
    void fillBuffer() @trusted
    {
        size_t nread;
        while (nread == 0 && !_eof)
        {
            // FIXME: timeout support
            _readEvent.wait(FileDescriptorEvent.Trigger.read);
            nread = readInto(_buffer[]);
        }
        _bufAvailable = _buffer[0 .. nread];
    }

    this(int fd) @trusted
    {
        // FIXME: Write support
        _fd = fd;
        _readEvent = createFileDescriptorEvent(_fd, FileDescriptorEvent.Trigger.read);
        //_writeEvent = createFileDescriptorEvent(_fd, FileDescriptorEvent.Trigger.write);
    }

public:
    /**
     * 
     */
    final BaudRate baudrate() @property const @trusted
    {
        auto options = getOptions();
        auto speed = cfgetispeed(&options);
        enforce(speed == cfgetospeed(&options), "Unknown speed value");

        return getFromPosixSpeed(speed);
    }

    /**
     * 
     * Throws: ParameterException if the baudrate could not be set
     */
    final void baudrate(BaudRate value) @property @trusted
    {
        auto options = getOptions();

        auto speed = value.getPosixSpeed();
        cfsetispeed(&options, speed);
        cfsetospeed(&options, speed);

        BaudRate newValue;
        if(!setOptions(options) || collectException(this.baudrate, newValue) !is null || newValue != value)
            throw new ParameterException(ParameterException.Type.baudRate);
    }

    /**
     * 
     */
    final Parity parity() @property const @safe
    {
        auto options = getOptions();
        if (options.c_cflag & PARENB)
        {
            if (options.c_cflag & PARODD)
                return Parity.odd;
            else
                return Parity.even;
        }
        else
            return Parity.none;
    }
    
    /**
     * 
     * Throws: ParameterException if the parity could not be set
     */
    final void parity(Parity value) @property @safe
    {
        auto options = getOptions();
        
        final switch(value)
        {
            case Parity.none:
                options.c_cflag &= ~PARENB;
                break;
            case Parity.even:
                options.c_cflag |= PARENB;
                options.c_cflag &= ~PARODD;
                break;
            case Parity.odd:
                options.c_cflag |= PARENB;
                options.c_cflag |= PARODD;
                break;
        }

        Parity newValue;
        if(!setOptions(options) || collectException(this.parity, newValue) !is null || newValue != value)
            throw new ParameterException(ParameterException.Type.parity);
    }

    /**
     * 
     */
    final StopBits stopBits() @property const @safe
    {
        auto options = getOptions();

        if (options.c_cflag & CSTOPB)
            return StopBits.two;
        else
            return StopBits.one;
    }
    
    /**
     * 
     * Throws: ParameterException if the number of stop bits could not be set
     */
    final void stopBits(StopBits value) @property @safe
    {
        auto options = getOptions();
        
        final switch(value)
        {
            case StopBits.one:
                options.c_cflag &= ~CSTOPB;
                break;
            case StopBits.two:
                options.c_cflag |= CSTOPB;
                break;
        }

        StopBits newValue;
        if(!setOptions(options) || collectException(this.stopBits, newValue) !is null || newValue != value)
            throw new ParameterException(ParameterException.Type.stopBits);
    }

    /**
     * 
     */
    final DataBits dataBits() @property const @safe
    {
        auto options = getOptions();

        // Get all size bits
        auto masked = options.c_cflag & CSIZE;
        // Remove all size bits we know of
        masked &= ~(CS5 | CS6 | CS7 | CS8);
        // If there are still set bits, this is a size we don't know of (CS9, CS4, ...)
        // (obviously, the druntime CSIZE definition needs to be correct for this to work)
        enforce(masked == 0, "Unsupported number of DataBits!");

        // Now check all known values
        if ((options.c_cflag & CSIZE) == CS5)
            return DataBits.five;
        else if ((options.c_cflag & CSIZE) == CS6)
            return DataBits.six;
        else if ((options.c_cflag & CSIZE) == CS7)
            return DataBits.seven;
        else if ((options.c_cflag & CSIZE) == CS8)
            return DataBits.eight;
        else
            throw new Exception("Unsupported number of DataBits!");
    }
    
    /**
     * 
     * Throws: ParameterException if the number of data bits could not be set
     */
    final void dataBits(DataBits value) @property @safe
    {
        auto options = getOptions();
        
        options.c_cflag &= ~(CSIZE);
        final switch(value)
        {
            case DataBits.five:
                options.c_cflag |= CS5;
                break;
            case DataBits.six:
                options.c_cflag |= CS6;
                break;
            case DataBits.seven:
                options.c_cflag |= CS7;
                break;
            case DataBits.eight:
                options.c_cflag |= CS8;
                break;
        }

        DataBits newValue;
        if(!setOptions(options) || collectException(this.dataBits, newValue) !is null || newValue != value)
            throw new ParameterException(ParameterException.Type.dataBits);
    }

    /**
     * TODO: hardware flow control
     */
    final FlowControl flowControl() @property const @safe
    {
        auto options = getOptions();

        if((options.c_iflag & IXON) && (options.c_iflag & IXOFF))
            return FlowControl.software;
        else if(!(options.c_iflag & IXON) && !(options.c_iflag & IXOFF))
            return FlowControl.none;
        else
            throw new Exception("Unsupported flow control method!");
    }
    
    /**
     * 
     * Throws: ParameterException if the flow control type could not be set
     */
    final void flowControl(FlowControl value) @property @safe
    {
        auto options = getOptions();

        // We do not want to have any character start flowing
        options.c_iflag &= ~(IXANY);
        if(value == FlowControl.none)
            options.c_iflag &= ~(IXON | IXOFF);
        else if(value == FlowControl.software)
            options.c_iflag |= (IXON | IXOFF);

        FlowControl newValue;
        if(!setOptions(options) || collectException(this.flowControl, newValue) !is null || newValue != value)
            throw new ParameterException(ParameterException.Type.flowControl);
    }

    /**
     * Standard vibe.d ConnectionStream interface
     */
    bool dataAvailableForRead() @property
    {
        return _bufAvailable.length != 0;
    }

    ///ditto
    bool empty() @property
    {
        // If we're not at eof yet, but buffer is empty we need to read to determine whether
        // there is more data
        if(_bufAvailable.length == 0 && !_eof)
            fillBuffer();

        if (_eof)
            return true;
        else
            return false;
    }

    ///ditto
    ulong leastSize() @property
    {
        if (this.empty)
            return 0;

        return _bufAvailable.length;
    }

    ///ditto
    const(ubyte)[] peek()
    {
        return _bufAvailable;
    }

    ///ditto
    void read(ubyte[] dst)
    {
        if (_bufAvailable.length >= dst.length)
        {
            dst[] = _bufAvailable[0 .. dst.length];
            _bufAvailable = _bufAvailable[dst.length .. $];
            return;
        }
        else
        {
            // First consume buffer
            size_t nread = _bufAvailable.length;
            dst[0 .. _bufAvailable.length] = _bufAvailable[];
            _bufAvailable = [];


            while (nread < dst.length && !_eof)
            {
                // FIXME: timeout support
                _readEvent.wait(FileDescriptorEvent.Trigger.read);
                nread += readInto(dst[nread .. $]);
            }

            enforce(nread == dst.length, "EOF reached before completely filling buffer!");
        }
    }

    ///ditto
    void write(const(ubyte[]) bytes)
    {
        const(ubyte)[] mBytes = bytes;
        while (mBytes.length != 0)
        {
            _writeEvent.wait(FileDescriptorEvent.Trigger.write);
            size_t written = writeFrom(mBytes[]);
            mBytes = mBytes[written .. $];
        }
    }

    ///ditto
    void write(InputStream stream, ulong nbytes = 0)
    {
        writeDefault(stream, nbytes);
    }

    ///ditto
    void flush()
    {
        errnoEnforce(tcflush(_fd, TCOFLUSH) != -1, "Couldn't flush serial port buffer");
    }

    ///ditto
    void finalize()
    {
        flush();
        close();
    }

    ///ditto
    void close()
    {
        if (_fd != -1)
        {
            // Can't really handle errors here anyway (FD is in undefined state after error)
            posix_close(_fd);
            _fd = -1;
        }
    }

    ///ditto
    bool connected() const @property
    {
        return _fd != -1;
    }

    ///ditto
    bool waitForData(Duration timeout)
    {
        if (_buffer.length != 0)
            return true;
        else
            return _readEvent.wait(timeout, FileDescriptorEvent.Trigger.read);
    }
}

/**
 * TODO: Read/Write only modes?
 */
SerialConnection connectSerial(string file, BaudRate rate, Parity parity = Parity.none,
    StopBits stopBits = StopBits.one, DataBits dataBits = DataBits.eight, FlowControl flow = FlowControl.none) @trusted
{
    import std.string : toStringz;
    enforce(parity == Parity.none, "Using parity bits is not yet supported");
    enforce(flow != FlowControl.hardware, "Hardware flow control is not yet supported");

    // Open file descriptor
    auto fd = posix_open(file.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    errnoEnforce(fd != -1, "Couldn't open serial port");

    scope(failure)
    {
        // Can't really handle errors here anyway (FD is in undefined state after error)
        posix_close(fd);
    }

    auto conn = new SerialConnection(fd);
    conn.setBaseParams();
    conn.baudrate = rate;
    conn.parity = parity;
    conn.stopBits = stopBits;
    conn.dataBits = dataBits;
    conn.flowControl = flow;

    return conn;
}

private speed_t getPosixSpeed(BaudRate rate) @safe pure
{
    final switch(rate)
    {
        case BaudRate.B50:
            return B50;
        case BaudRate.B75:
            return B75;
        case BaudRate.B110:
            return B110;
        case BaudRate.B134:
            return B134;
        case BaudRate.B150:
            return B150;
        case BaudRate.B200:
            return B200;
        case BaudRate.B300:
            return B300;
        case BaudRate.B600:
            return B600;
        case BaudRate.B1200:
            return B1200;
        case BaudRate.B1800:
            return B1800;
        case BaudRate.B2400:
            return B2400;
        case BaudRate.B4800:
            return B4800;
        case BaudRate.B9600:
            return B9600;
        case BaudRate.B19200:
            return B19200;
        case BaudRate.B38400:
            return B38400;
        case BaudRate.B57600:
            throw new Exception("BaudRate B57600 not supported in druntime");
        case BaudRate.B76800:
            throw new Exception("BaudRate B76800 not supported in druntime");
        case BaudRate.B115200:
            throw new Exception("BaudRate B115200 not supported in druntime");
    }
}

private BaudRate getFromPosixSpeed(speed_t rate) @safe pure
{
    switch(rate)
    {
        case B50:
            return BaudRate.B50;
        case B75:
            return BaudRate.B75;
        case B110:
            return BaudRate.B110;
        case B134:
            return BaudRate.B134;
        case B150:
            return BaudRate.B150;
        case B200:
            return BaudRate.B200;
        case B300:
            return BaudRate.B300;
        case B600:
            return BaudRate.B600;
        case B1200:
            return BaudRate.B1200;
        case B1800:
            return BaudRate.B1800;
        case B2400:
            return BaudRate.B2400;
        case B4800:
            return BaudRate.B4800;
        case B9600:
            return BaudRate.B9600;
        case B19200:
            return BaudRate.B19200;
        case B38400:
            return BaudRate.B38400;
        default:
            throw new Exception("Unknown speed value");
    }

    // Can't happen
    assert(0);
}
