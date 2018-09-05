#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <string>
#include <boost/thread/thread.hpp>
#include <chrono>
#include <algorithm>

#define PNG 0
#define BMP 1

using boost::asio::ip::tcp;
using namespace std;
const int dwidth = 512;
const int dheight = 424;

unsigned char colorBuffer[dwidth*dheight * 4];
USHORT depthsBuffer[dwidth*dheight];

int NUMSENSORS;
int FORMAT;
bool RECORDSKELETON;
USHORT LIMIT;
std::vector<shared_ptr<boost::asio::io_service> > services;
std::vector<shared_ptr<tcp::socket> > sockets;
vector<string> ips;
vector<string> ports;


void loadConfig(){
	boost::property_tree::ptree pt;
	boost::property_tree::ini_parser::read_ini("config.ini", pt);
	NUMSENSORS = pt.get<int>("NUMSENSORS");
	FORMAT = pt.get<int>("FORMAT");
	LIMIT = pt.get<USHORT>("LIMIT");
	for (int i = 0; i < NUMSENSORS; i++){
		stringstream ssa, ssb;
		ssa << "IP" << i;
		ssb << "PORT" << i;
		ips.push_back(pt.get<std::string>(ssa.str()));
		ports.push_back(pt.get<std::string>(ssb.str()));
	}
	RECORDSKELETON = pt.get<int>("RECORDSKELETON") == 1 ? true : false;
	if (RECORDSKELETON) NUMSENSORS++;
	ips.push_back(pt.get<std::string>("IPSKELETON"));
	ports.push_back(pt.get<std::string>("PORTSKELETON"));
	
}



string ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos);
}

void writeDepthFile(string name){
	HANDLE hFile;
	std::ostringstream patha;
	string path = ExePath();
	patha << path << "\\"<< "Output\\depth\\" <<name;
	std::string sa = patha.str();
	std::wstring stempa = std::wstring(sa.begin(), sa.end());
	LPCWSTR swa = stempa.c_str();
	hFile = CreateFileW(swa, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);
	DWORD dwBytesWritten = 0;
	WriteFile(hFile, &depthsBuffer, sizeof(depthsBuffer), &dwBytesWritten, NULL);
	CloseHandle(hFile);
}




HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR path)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = { 0 };

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes = 1;                         // Default
	bmpInfoHeader.biSizeImage = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = { 0 };

	bfh.bfType = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers


	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(path, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	// Return if error opening file
	if (NULL == hFile)
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the RGB Data
	if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}




unsigned char tempCBuffer[1024];
USHORT tempDBuffer[dwidth*dheight];
void getColor(int id){
	int i = 0;
		printf("writing color %d\n", id);
		for (;;)
		{
			boost::system::error_code error;
			size_t len = sockets[id]->read_some(boost::asio::buffer(tempCBuffer, 1024 * sizeof(unsigned char)), error);

			if (error == boost::asio::error::eof)
				break; // Connection closed cleanly by peer.
			else if (error){
				std::cout << error.message();
				throw boost::system::system_error(error); // Some other error.

			}

			for (int k = 0; k < len; k++){
				colorBuffer[i++] = tempCBuffer[k];
			}
			if (i >= dwidth*dheight * 4){
				std::cout << "Received " << i << "bytes\n";
				

				break;
			}

		}
}

int filterSize = 7;
void filterPixelMedian()
{
	int offsetplus = (int)std::floor(filterSize / 2.0);
	int offsetless = (int)std::ceil(filterSize / 2.0) - 1;
	
	USHORT orig[dwidth*dheight];
	for (int a = 0; a < dwidth*dheight; a++) orig[a] = depthsBuffer[a];
	USHORT *values = new USHORT[filterSize * filterSize];

	
	for (int x = 0; x < dheight; x++){
		for (int y = 0; y < dwidth; y++){
			int k = 0;
			for (int i = 0; i < filterSize; i++)
			{
				for (int j = 0; j < filterSize; j++)
				{
					int newx = (x - offsetless + i);
					int newy = (y - offsetless + j);
					//extending borders for convolution
					newx = newx < 0 ? 0 : newx;
					newx = newx >= dwidth ? dwidth - 1 : newx;
					newy = newy < 0 ? 0 : newy;
					newy = newy >= dheight ? dheight - 1 : newy;
					int idx = (newx * dwidth) + newy;
					values[k++] = orig[idx];
				}
			}
			int halfindex = (int)std::floor(filterSize * filterSize / 2.0);
			//orderby values.
			//get the one in 
			std::sort(values, values + (filterSize*filterSize));
			int idx2 = (x * dwidth) + y;
			depthsBuffer[idx2] = values[halfindex];
		}
	}
	delete values;
}
void getDepth(int id,bool limit){
	int i = 0;
		printf("writing depth %d\n", id);
		int bytes = 0;
		for (;;)
		{
			boost::system::error_code error;

			size_t len = sockets[id]->read_some(boost::asio::buffer(tempDBuffer, dwidth*dheight*sizeof(USHORT)), error);
			bytes += len;
			if (error == boost::asio::error::eof)
				break; // Connection closed cleanly by peer.
			else if (error)
				throw boost::system::system_error(error); // Some other error.
			int k = 0;
			for (int j=0; j <len; j+=sizeof(USHORT)){
				if (!limit || tempDBuffer[k] < LIMIT)
					depthsBuffer[i++] = tempDBuffer[k++];
				else{
					depthsBuffer[i++] = 0;
					k++;
				}
			}
			if (i >= dheight*dwidth){
				std::cout << "Received " << bytes << "bytes\n";
				break;
			}
		}
}
void connectToServers(){
	for (int j = 0; j < NUMSENSORS; j++){

		services.push_back(shared_ptr<boost::asio::io_service>(new boost::asio::io_service()));
		tcp::resolver resolver(*services[j]);
		tcp::resolver::query query(ips[j], ports[j]);
		tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
		sockets.push_back(shared_ptr<tcp::socket>(new tcp::socket(*services[j])));
		boost::asio::connect(*sockets[j], endpoint_iterator);
	}
}

void sendTimes(){
	std::chrono::high_resolution_clock::time_point  t1 = std::chrono::high_resolution_clock::now();
	std::cout << "Function called at:"<< t1.time_since_epoch().count() << "\n";
	std::chrono::seconds d(5);
	t1 += d; 
	std::cout << "Capture happening at:" << t1.time_since_epoch().count();
	stringstream ss;
	ss << t1.time_since_epoch().count();
	for (int j = 0; j < NUMSENSORS; j++){
		boost::system::error_code error;
		boost::asio::write(*sockets[j], boost::asio::buffer(ss.str()), error);
	}
}

void sendRequests(string message){

	for (int j = 0; j < NUMSENSORS; j++){
		boost::system::error_code error;
		boost::asio::write(*sockets[j], boost::asio::buffer(message),boost::asio::transfer_all(), error);
	}
}

void sendOk(int id){
	boost::system::error_code error;
	boost::asio::write(*sockets[id], boost::asio::buffer("OK"), boost::asio::transfer_all(), error);
}
void readOks(){
	char bufok[12];
	boost::system::error_code error;
	for (int j = 0; j < NUMSENSORS; j++){
		size_t len = (*sockets[j]).read_some(boost::asio::buffer(bufok), error);
		printf("read %d bytes ok\n", len);
	}
}

void saveDepth(int id, bool median){

	if (median){
		filterPixelMedian();
	}
	stringstream filename;
	filename << "depthPlayerdata0," << id;
	writeDepthFile(filename.str());
}

void saveColor(int id){
	stringstream filename;

	filename << "Output\\color\\color0," << id << ".bmp";
	std::string sb = filename.str();
	std::wstring stemp = std::wstring(sb.begin(), sb.end());
	LPCWSTR sw = stemp.c_str();
	SaveBitmapToFile(colorBuffer, dwidth, dheight, 32, sw);

}
int main(int argc, char* argv[])
{
	string message;
	getline(std::cin, message);
	loadConfig();
	connectToServers();
	sendRequests(message);
	boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	readOks();
	sendTimes();
	boost::this_thread::sleep(boost::posix_time::milliseconds(4000));
	
	if (message == "snap" || message == "snaplong"){
		//get responses
	//	bool median = message == "snaplong" ? true : false;
		bool limit = LIMIT == 0 ? false : true;
		if (RECORDSKELETON) NUMSENSORS--;
		for (int id = 0; id < NUMSENSORS; id++){
			getDepth(id,limit);
			sendOk(id);
			getColor(id);
			sendOk(id);
			saveDepth(id, false);
			saveColor(id);
		}
	}
	
	return 0;
}