#define _WINSOCK_DEPRECATED_NO_WARNINGS
#define WIN32_LEAN_AND_MEAN
#include <cmath>
#include <cstdio>
#include <Ole2.h>
#include <sstream>
#include <time.h>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <Windows.h>

using boost::asio::ip::tcp;
using boost::asio::ip::udp;
using namespace std;

bool rec = false;
boost::mutex semaforo;
ofstream file;
int broadcastPort = 0;
string computerTimeSyncName;

string receivedSkeleton = "0";

class timer {
private:
	unsigned long begTime;
public:
	void start() {
		begTime = clock();
	}

	unsigned long elapsedTime() {
		return ((unsigned long)clock() - begTime) / CLOCKS_PER_SEC;
	}

	bool isTimeout(unsigned long seconds) {
		return seconds >= elapsedTime();
	}
};

void loadConfig() {
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("config.ini", pt);
	computerTimeSyncName = pt.get<std::string>("timesync");
	broadcastPort = pt.get<int>("broadcastPort");
}

class udp_server
{
public:
	udp_server(boost::asio::io_service& io_service)
		: socket_(io_service, udp::endpoint(boost::asio::ip::address_v4::any(), broadcastPort))
	{
		start_receive();
		std::cout << "listening at port " << broadcastPort << "\n";
	}

private:
	void start_receive()
	{
		socket_.async_receive_from(
			boost::asio::buffer(recv_buffer), remote_endpoint_,
			boost::bind(&udp_server::handle_receive, this,
				boost::asio::placeholders::error,
				boost::asio::placeholders::bytes_transferred));
	}

	void handle_receive(const boost::system::error_code& error,
		std::size_t bytes_transferred)
	{

		if (!error || error == boost::asio::error::message_size)
		{
			if (rec) {
				semaforo.lock();
				receivedSkeleton = std::string(recv_buffer.begin(), recv_buffer.begin() + bytes_transferred);
				semaforo.unlock();
			}
		}
		else {
			std::cout << error.message();
		}
		start_receive();

	}

	void handle_send(boost::shared_ptr<std::string> /*message*/,
		const boost::system::error_code& /*error*/,
		std::size_t /*bytes_transferred*/)
	{
	}

	udp::socket socket_;
	udp::endpoint remote_endpoint_;
	boost::array<char, 2048> recv_buffer;
};




string ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos);
}



void writeThreadTask() {
	std::cout << "started writing thread" << endl;
	auto lastWrite = std::chrono::high_resolution_clock::now();
	auto frequency = std::chrono::microseconds(33333);
	while (rec)
	{
		auto t1 = std::chrono::high_resolution_clock::now();
		if (t1 - lastWrite> frequency) {
			lastWrite = t1;
			semaforo.lock();
			file << receivedSkeleton << "\n";
			file.flush();
			semaforo.unlock();
		}

	}
}

void record(bool start, bool single, bool acum, bool reset) {

	semaforo.lock();
	rec = start;
	if (start) {
		stringstream ss;
		std::cout << "will create file \n";
		auto t1 = std::chrono::high_resolution_clock::now();
		unsigned long long nowtime = t1.time_since_epoch().count();
		ss << ExePath() << "\\" << nowtime << ".txt";
		file.open(ss.str());
		std::cout << "file path:" << ss.str() << "\n";
		std::cout << "Created!\n";
		boost::thread thread(writeThreadTask);
	}
	else {
		file.flush();
		file.close();
	}
	semaforo.unlock();
	printf("recording");
}


std::chrono::high_resolution_clock::time_point lastCap;
std::chrono::high_resolution_clock::time_point lastSec;
bool dropped = false;



void kinectThreadTask() {
	std::cout << "ready to receive" << endl;
	boost::asio::io_service io_service;
	udp_server server(io_service);
	io_service.run();
	return;
}

/*NETWORKING PART START */


class tcp_connection
	: public boost::enable_shared_from_this<tcp_connection>
{
public:
	typedef boost::shared_ptr<tcp_connection> pointer;
	static pointer create(boost::asio::io_service& io_service)
	{
		return pointer(new tcp_connection(io_service));
	}

	tcp::socket& socket()
	{
		return socket_;
	}



	void syncComputerClocks() {
		stringstream ss1;
		ss1 << "net time \\\\" << computerTimeSyncName << " /set /yes";
		system(ss1.str().c_str());

	}

	void netStartCap() {
		record(true, false, false, true);
	}

	void sendOk() {
		boost::system::error_code error;
		boost::asio::write(socket_, boost::asio::buffer("OK"), boost::asio::transfer_all(), error);
	}

	void netStopCap() {
		record(false, false, false, false);
	}

	void start()
	{
		//syncComputerClocks();

		//Read request
		char bufReq[32];
		char bufTstp[256];
		boost::system::error_code error;
		size_t bytesRead = socket_.read_some(boost::asio::buffer(bufReq), error);
		bufReq[bytesRead] = '\0';
		string sReq(bufReq);

		sendOk();

		//Read timestamp
		bytesRead = socket_.read_some(boost::asio::buffer(bufTstp), error);
		bufTstp[bytesRead] = '\0';
		string stamp(bufTstp);

		stringstream ss;
		ss << stamp;
		unsigned long long captime;
		ss >> captime;
		auto t1 = std::chrono::high_resolution_clock::now();
		unsigned long long nowtime = t1.time_since_epoch().count();
		std::cout << "Request " << sReq << " now " << nowtime << " cap " << captime << "\n";
		unsigned long long sleeptime = captime - nowtime;
		sleeptime = sleeptime / 1000;
		std::cout << "sleeping for " << sleeptime << "ms \n";

		boost::this_thread::sleep(boost::posix_time::microseconds(sleeptime));

		if (sReq == "start") {
			netStartCap();
		}
		if (sReq == "stop") {
			netStopCap();
		}
	}

private:
	tcp_connection(boost::asio::io_service& io_service)
		: socket_(io_service)
	{

	}

	void handle_write(const boost::system::error_code& /*error*/,
		size_t /*bytes_transferred*/)
	{
	}

	tcp::socket socket_;
};

class tcp_server
{
public:
	tcp_server(boost::asio::io_service& io_service)
		: acceptor_(io_service, tcp::endpoint(tcp::v4(), 4242))
	{
		start_accept();
	}

private:
	void start_accept()
	{
		tcp_connection::pointer new_connection =
			tcp_connection::create(acceptor_.get_io_service());

		acceptor_.async_accept(new_connection->socket(),
			boost::bind(&tcp_server::handle_accept, this, new_connection,
				boost::asio::placeholders::error));
	}

	void handle_accept(tcp_connection::pointer new_connection,
		const boost::system::error_code& error)
	{
		if (!error)
		{
			new_connection->start();
		}

		start_accept();
	}

	tcp::acceptor acceptor_;
};

void serverThreadTask()
{
	try
	{
		std::cout << "Server thread started" << endl;
		boost::asio::io_service io_service;
		tcp_server server(io_service);
		io_service.run();
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return;
}
/*NETWORKING PART FINISH*/


int main(int argc, char* argv[]) {
	loadConfig();
	std::cout << "Config loaded" << endl;
	boost::thread thread1(serverThreadTask);
	boost::thread thread2(kinectThreadTask);
	loadConfig();
	thread1.join();
	thread2.join();
	return 0;
}