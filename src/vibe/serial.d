/**
 * 
 */
module vibe.serial;

import core.sys.posix.unistd, core.sys.posix.fcntl, core.sys.posix.termios, core.stdc.errno;
import std.exception;
import vibe.d;


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

    size_t readInto(ubyte[] buf)
    {
        assert(buf.length > 0);

        auto res = core.sys.posix.unistd.read(_fd, buf.ptr, buf.length);

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

    size_t writeFrom(const(ubyte)[] buf)
    {
        assert(buf.length > 0);

        auto res = core.sys.posix.unistd.write(_fd, buf.ptr, buf.length);
        
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

    this(int fd)
    {
        // FIXME: Write support
        _fd = fd;
        _readEvent = createFileDescriptorEvent(_fd, FileDescriptorEvent.Trigger.read);
        _writeEvent = createFileDescriptorEvent(_fd, FileDescriptorEvent.Trigger.write);
    }

public:
    @property bool dataAvailableForRead()
    {
        return _bufAvailable.length != 0;
    }

    //FIXME: Have to read in empty: lastSize may not return 0 if empty returned false before
    @property bool empty()
    {
        return _eof && _bufAvailable.length == 0;
    }

    @property size_t leastSize()
    {
        if (_bufAvailable.length == 0)
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

        return _bufAvailable.length;
    }

    const(ubyte)[] peek()
    {
        return _bufAvailable;
    }

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

    void write(InputStream stream, size_t nbytes = 0)
    {
        writeDefault(stream, nbytes);
    }

    void flush()
    {
        errnoEnforce(tcflush(_fd, TCOFLUSH) != -1, "Couldn't flush serial port buffer");
    }

    void finalize()
    {
        flush();
        close();
    }

    void close()
    {
        if (_fd != -1)
        {
            // Can't really handle errors here anyway (FD is in undefined state after error)
            core.sys.posix.unistd.close(_fd);
            _fd = -1;
        }
    }
    bool connected() const @property
    {
        return _fd != -1;
    }
    bool waitForData(Duration timeout)
    {
        //FIXME
        return false;
    }
}

/**
 * TODO: Read/Write only modes?
 */
SerialConnection connectSerial(string file, BaudRate rate, Parity parity = Parity.none,
    StopBits stopBits = StopBits.one, DataBits dataBits = DataBits.eight, FlowControl flow = FlowControl.none)
{
    import std.string : toStringz;
    enforce(parity == Parity.none, "Using parity bits is not yet supported");
    enforce(flow != FlowControl.hardware, "Hardware flow control is not yet supported");

    // Open file descriptor
    auto fd = open(file.toStringz(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    errnoEnforce(fd != -1, "Couldn't open serial port");

    scope(failure)
    {
        // Can't really handle errors here anyway (FD is in undefined state after error)
        close(fd);
    }


    // Set serial options
    termios options;
    errnoEnforce(tcgetattr(fd, &options) != -1, "Couldn't get terminal options");
    options.c_cflag |= CLOCAL | CREAD;
    options.c_cflag &= ~(HUPCL);

    // Baudrate
    cfsetispeed(&options, rate.getPosixSpeed());
    cfsetospeed(&options, rate.getPosixSpeed());

    // Parity
    final switch(parity)
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

    // Set number of stop bits
    final switch(stopBits)
    {
        case StopBits.one:
            options.c_cflag &= ~CSTOPB;
            break;
        case StopBits.two:
            options.c_cflag |= CSTOPB;
            break;
    }

    // Set number of data bits
    options.c_cflag &= ~(CSIZE);
    final switch(dataBits)
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

    // Flow control
    // Only allow explicit start flow
    options.c_iflag &= ~(IXANY);
    if(flow == FlowControl.none)
        options.c_iflag &= ~(IXON | IXOFF);
    else if(flow == FlowControl.software)
        options.c_iflag |= (IXON | IXOFF);


    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG | ECHOK | ECHONL | IEXTEN);
    // No postprocessing
    options.c_oflag &= ~OPOST;

    // ignore break condition
    options.c_iflag &= ~(IXON | IXOFF | IXANY | BRKINT);
    options.c_iflag |= IGNBRK;

    // Now apply the serial port settings
    errnoEnforce(tcsetattr(fd, TCSANOW, &options) != -1, "Couldn't set terminal options");

    // Verify options
    termios options2;
    errnoEnforce(tcgetattr(fd, &options2) != -1, "Couldn't verify terminal options");
    //FIXME: Maybe check only baudrate, stopbits, ... manually?
    errnoEnforce(options == options2, "Couldn't set all terminal options");

    return new SerialConnection(fd);
}

private speed_t getPosixSpeed(BaudRate rate)
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