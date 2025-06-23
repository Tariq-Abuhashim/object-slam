// Copyright (c) 2023 Mission Systems Pty Ltd

#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/csv/stream.h>
#include <comma/io/select.h>
#include <comma/io/stream.h>
#include <comma/visiting/traits.h>
#include <boost/date_time/posix_time/posix_time.hpp>

void usage( bool )
{
    std::cerr << "\nSample application for reading binary data streams";
    std::cerr << "\n";
    std::cerr << "\nMakes some major assumptions about the input data";
    std::cerr << "\n";
    std::cerr << "\nUsage: sample-reader-tcp <list-of-ports>";
    std::cerr << "\n";
    std::cerr << "\nOptions:";
    std::cerr << "\n    --help,-h:     display this help message and exit";
    std::cerr << "\n    --verbose,-v:  more output";
    std::cerr << "\n";
    std::cerr << "\nExample:";
    std::cerr << "\n    sample-reader-tcp 'tcp:localhost:4000;binary=t,2uw,2ui,uw,3d,3uw,3d' 'tcp:localhost:4001;binary=t,3ui,s[15197952]'";
    std::cerr << "\n" << std::endl;
}

struct timestamped_data
{
    boost::optional< boost::posix_time::ptime > timestamp;
    timestamped_data() {}
    timestamped_data( const boost::posix_time::ptime& timestamp ) : timestamp( timestamp ) {}
};

namespace comma { namespace visiting {

template <> struct traits< timestamped_data >
{
    template < typename K, typename V > static void visit( const K&, const timestamped_data& p, V& v ) { v.apply( "t", p.timestamp ); }
    template < typename K, typename V > static void visit( const K&, timestamped_data& p, V& v ) { v.apply( "t", p.timestamp ); }
};

} } // namespace comma { namespace visiting {

class stream
{
public:
    stream( const std::string& input )
    {
        std::vector< std::string > input_components = comma::split( input, ';' );
        port = input_components[0];
        format = comma::split( input_components[1], '=' )[1];
    }

    comma::io::file_descriptor fd() const { return ( *istream ).fd(); }

    void connect()
    {
        std::cerr << "connecting to " << port << std::endl;
        istream.reset( new comma::io::istream( port, comma::io::mode::binary ));
        std::cerr << "with fd " << fd() << std::endl;
    }

    std::size_t read( std::vector< char >& buffer )
    {
        std::size_t available = available_();
        if( available == 0 ) { return 0; }
        std::size_t size = std::min( available, buffer.size() );
        ( *istream )->read( &buffer[0], size );
        return ( *istream )->gcount() <= 0 ? 0 : ( *istream )->gcount();
    }

private:
    std::size_t available_()
    {
        std::size_t available_on_fd = ( *istream ).available_on_file_descriptor();
        std::streamsize ss = ( *istream )->rdbuf()->in_avail();
        return available_on_fd + ss;
    }

    std::string port;
    std::string format;
    std::unique_ptr< comma::io::istream > istream;
};

int process( const std::vector< std::string >& inputs )
{
    comma::io::select select;
    std::vector< stream* > streams;

    for( const std::string& input : inputs ) { streams.push_back( new stream( input ) ); }

    for( auto s : streams )
    {
        s->connect();
        select.read().add( *s );
    }

    comma::signal_flag is_shutdown;
    bool end_of_input = false;
    const timestamped_data* record = nullptr;

    std::vector< char > buffer( 65536 );

    while( !is_shutdown && !end_of_input )
    {
        if( select.check() )
        {
            for( auto& s : streams )
            {
                std::size_t bytes_read = s->read( buffer );
                if( bytes_read > 0 )
                {
                    std::cerr << "got " << bytes_read << " bytes fron fd " << s->fd() << std::endl;
                }
            }
        }
    }
}

int main( int ac, char** av )
{
    try
    {
        comma::command_line_options options( ac, av, usage );
        std::vector< std::string > ports = options.unnamed( "--help,-h,--verbose,-v", "-.*" );
        if( ports.size() > 0 )
        {
            return process( ports );
        }
        std::cerr << comma::verbose.app_name() << ": requires <list-of-ports>" << std::endl;
        return 1;
    }
    catch( std::exception& ex ) { std::cerr << comma::verbose.app_name() << ": " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << comma::verbose.app_name() << ": unknown exception" << std::endl; }
    return 1;
}
