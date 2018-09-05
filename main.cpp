
#define WIN32_LEAN_AND_MEAN
#include "glut.h"
#include "main.h"
#include <cmath>
#include <cstdio>
#include <Ole2.h>
#include <sstream>
#include <Kinect.h>
#include <time.h>
#include <chrono>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <boost/asio.hpp>
#include <Windows.h>
using boost::asio::ip::tcp;
using namespace std;

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

// OpenGL Variables
long depthToRgbMap[width*height * 2];
GLuint vboId;
GLuint cboId;
GLuint CtextureId;
GLuint DtextureId;
GLubyte rgbdata[width*height * 4];
GLubyte depthdata[dwidth*dheight * 4];
ColorSpacePoint colorPoints[dwidth*dheight];
DepthSpacePoint depthPoints[width*height];
GLubyte iniColorBuffer[width*height * sizeof(RGBQUAD)];
IKinectSensor*          sensor;
IDepthFrameReader*      depthFrameReader;
IBodyIndexFrameReader*  bodyIndexFrameReader;
IColorFrameReader*		colorFrameReader;
int separatePlayers = 0;
int frameCount = -1;
int secCount = 0;
int acumCount = 0;
timer tIter;

/// SHARED VARIABLES
GLubyte colorBuffer[dwidth*dheight * 4];
USHORT depthsBuffer[dwidth*dheight];
bool rec = false;
bool snap = false;

boost::mutex semaforo;

string ExePath() {
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	string::size_type pos = string(buffer).find_last_of("\\/");
	return string(buffer).substr(0, pos);
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

bool initKinect(){
	GetDefaultKinectSensor(&sensor);

	if (sensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;
		IBodyIndexFrameSource* pBodyIndexFrameSource = NULL;
		IColorFrameSource* pColorFrameSource = NULL;
	
		sensor->Open();
		sensor->get_DepthFrameSource(&pDepthFrameSource);
		sensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		sensor->get_ColorFrameSource(&pColorFrameSource);

		pDepthFrameSource->OpenReader(&depthFrameReader);
		pBodyIndexFrameSource->OpenReader(&bodyIndexFrameReader);
		pColorFrameSource->OpenReader(&colorFrameReader);

		SafeRelease(pDepthFrameSource);
		SafeRelease(pBodyIndexFrameSource);
		SafeRelease(pColorFrameSource);
	}

	if (!sensor)
	{
		return false;
	}

	return true;
}

void record(bool start, bool single,bool acum,bool reset){

	if (reset){
		if (start){
			semaforo.lock();
			sensor->Open();
			semaforo.unlock();
		}
		else{
			semaforo.lock();
			sensor->Close();
			semaforo.unlock();
		}
	}
	
	sensor->Open();
	semaforo.lock();
	rec = start;
	snap = single;
	if (acum)
		acumCount = 60;
	semaforo.unlock();
	printf("recording");
	frameCount = -1;
	
}

void getDepthImageData(GLubyte* dest){
	IDepthFrame* pDepthFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	depthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (pDepthFrame == NULL) return;
	bodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	if (pBodyIndexFrame == NULL) {
		SafeRelease(pDepthFrame);
		return;
	}
	USHORT nDepthMin= 0;
	USHORT nDepthMax = 0;
	UINT nBufferSize = 0;
	UINT nBodyBufferSize = 0;
	UINT16 *pBuffer = NULL;
	BYTE *pBodyBuffer = NULL;

	pDepthFrame->get_DepthMinReliableDistance(&nDepthMin);
	pDepthFrame->get_DepthMaxReliableDistance(&nDepthMax);

	pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
	pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyBufferSize, &pBodyBuffer);



	GLubyte* pRGBX = dest;
	HANDLE hFile = NULL;
	HANDLE hFileb = NULL;
	int j = 0;
	
	// end pixel is start + width*height - 1
	const UINT16* pBufferEnd = pBuffer + (dwidth * dheight);

	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;
		BYTE player = *pBodyBuffer;
		//BYTE intensity = static_cast<BYTE>((depth >= nDepthMin) && (depth <= nDepthMax) ? (depth % 256) : 0);
		BYTE intensity = depth % 256;

		if (player != 255)
			*pRGBX++ = 0;
		else
			*pRGBX++ = intensity;

		*pRGBX++= intensity;
		*pRGBX++ = intensity;
		*pRGBX++ = 0xff;
		
		++pBuffer;
		++pBodyBuffer;

		
	}
	
	SafeRelease(pDepthFrame);
	SafeRelease(pBodyIndexFrame);
}

void getRgbImageData(GLubyte* dest){
	IColorFrame* pColorFrame = NULL;
	colorFrameReader->AcquireLatestFrame(&pColorFrame);

	if (pColorFrame == NULL) return;

	UINT nBufferSize = 0;
	GLubyte*pBuffer = NULL;

	pBuffer = dest;
	nBufferSize = width * height * sizeof(RGBQUAD);
	pColorFrame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(pBuffer), ColorImageFormat_Bgra);
	SafeRelease(pColorFrame);
}

bool getandrecordData(){
	IColorFrame* pColorFrame = NULL;
	colorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (pColorFrame == NULL) return false;

	IDepthFrame* pDepthFrame = NULL;
	depthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (pDepthFrame == NULL){
		SafeRelease(pColorFrame);
		return false;
	}

	IBodyIndexFrame* pBodyIndexFrame = NULL;
	bodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	if (pBodyIndexFrame == NULL) {
		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		return false;
	}
	
	USHORT nDepthMin = 0;
	USHORT nDepthMax = 0;
	UINT nBufferSize = 0;
	UINT nBodyBufferSize = 0;
	UINT16 *pBuffer = NULL;
	BYTE *pBodyBuffer = NULL;
	UINT colorBufferSize = width * height * sizeof(RGBQUAD);

	pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
	pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyBufferSize, &pBodyBuffer);
	pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(iniColorBuffer), ColorImageFormat_Bgra);

	//FILE WRITE VARIABLES
	
	ICoordinateMapper *mapper;
	sensor->get_CoordinateMapper(&mapper);
	int i = 0, j = 0, k = 0;
	mapper->MapDepthFrameToColorSpace(nBufferSize, pBuffer, nBufferSize, colorPoints);
	
	for (int m = 0; m < dheight; m++){
		for (int n = 0; n < dwidth; n++)
		{
			USHORT depth = *pBuffer;
			BYTE player = *pBodyBuffer;

			++pBuffer;
			++pBodyBuffer;

			semaforo.lock();
			if (player != 255 || !separatePlayers){
				if (acumCount>0){
					if (depthsBuffer[j] == 0){
						depthsBuffer[j++] = depth;
					}
				} else{
					depthsBuffer[j++] = depth;
				}
			}
			else{
				depthsBuffer[j++] = 0;
			}
			ColorSpacePoint p = colorPoints[k++];
		
			if (p.Y>= 0 && p.Y < height && p.X >= 0 && p.X < width && (player != 255 || !separatePlayers)){
				int idx = (int)((floor(p.Y) *(width * sizeof(RGBQUAD))) + ((floor(p.X) * sizeof(RGBQUAD))));
				if (acumCount <= 0 || (colorBuffer[i] == 0 && colorBuffer[i + 1] == 0 && colorBuffer[i + 2] == 0) 
					|| (colorBuffer[i] == 125 && colorBuffer[i + 1] == 125 && colorBuffer[i + 2] == 125)){
					colorBuffer[i++] = iniColorBuffer[idx++];
					colorBuffer[i++] = iniColorBuffer[idx++];
					colorBuffer[i++] = iniColorBuffer[idx++];
					colorBuffer[i++] = iniColorBuffer[idx++];
				}
				else{
					i += 4;
					idx += 4;
				}
			}
			else{
				colorBuffer[i++] = 125;
				colorBuffer[i++] = 125;
				colorBuffer[i++] = 125;
				colorBuffer[i++] = 0x00;
			}
			semaforo.unlock();
		}
	}
	SafeRelease(pDepthFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pColorFrame);
	if (--acumCount > 0)
		return false;
	else
		return true;
}


BYTE players[dwidth*dheight];
bool isPlayer(int idx, int rDist, int x, int y){
	BYTE player = players[idx];

	bool foot = (x > (dwidth / 3) && x < 2 * (dwidth / 3) && y >(5 * dheight / 6)) ? true : false;
	if (player != 255){
		return true;
	}
	else if (rDist == 0 || !foot){
		return false;
	}
	else{
		int limit = dwidth*dheight;
		int id1 = idx - dwidth - 1,
			id2 = idx - dwidth,
			id3 = idx - dwidth + 1,
			id4 = idx - 1,
			id5 = idx + 1,
			id6 = idx + dwidth - 1,
			id7 = idx + dwidth,
			id8 = idx + dwidth + 1;
		if (players[id1] != 255 && players[id1] + 1 < rDist){
			players[idx] = players[id1] + 1;
			return true;
		}
		if (players[id2] != 255 && players[id2] + 1 < rDist){
			players[idx] = players[id2] + 1;
			return true;
		}
		if (players[id3] != 255 && players[id3] + 1 < rDist){
			players[idx] = players[id3] + 1;
			return true;
		}
		if (players[id4] != 255 && players[id4] + 1 < rDist){
			players[idx] = players[id4] + 1;
			return true;
		}
		if (players[id5] != 255 && players[id5] + 1 < rDist){
			players[idx] = players[id5] + 1;
			return true;
		}
		return false;
	}
}
bool getandrecordDataToDisk(){
	IColorFrame* pColorFrame = NULL;
	colorFrameReader->AcquireLatestFrame(&pColorFrame);
	if (pColorFrame == NULL) return false;

	IDepthFrame* pDepthFrame = NULL;
	depthFrameReader->AcquireLatestFrame(&pDepthFrame);
	if (pDepthFrame == NULL){
		SafeRelease(pColorFrame);
		return false;
	}

	IBodyIndexFrame* pBodyIndexFrame = NULL;
	bodyIndexFrameReader->AcquireLatestFrame(&pBodyIndexFrame);
	if (pBodyIndexFrame == NULL) {
		SafeRelease(pDepthFrame);
		SafeRelease(pColorFrame);
		return false;
	}

	USHORT nDepthMin = 0;
	USHORT nDepthMax = 0;
	UINT nBufferSize = 0;
	UINT nBodyBufferSize = 0;
	UINT16 *pBuffer = NULL;
	BYTE *pBodyBuffer = NULL;
	UINT colorBufferSize = width * height * sizeof(RGBQUAD);

	pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
	pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyBufferSize, &pBodyBuffer);
	pColorFrame->CopyConvertedFrameDataToArray(colorBufferSize, reinterpret_cast<BYTE*>(iniColorBuffer), ColorImageFormat_Bgra);

	//FILE WRITE VARIABLES
	USHORT depths[BUFFERSIZE];
	USHORT depthsb[BUFFERSIZE];
	HANDLE hFile;
	HANDLE hFileb;

	int j = 0;
	std::ostringstream path;
	path << ExePath() << "\\Output\\color\\color" << secCount << ","<< frameCount << ".bmp";
	std::string s = path.str();
	std::wstring stemp = std::wstring(s.begin(), s.end());
	LPCWSTR sw = stemp.c_str();

	std::ostringstream patha;
	patha << ExePath() << "\\Output\\depthpl\\depthPlayerdata" <<secCount <<","<< frameCount;
	std::string sa = patha.str();
	std::wstring stempa = std::wstring(sa.begin(), sa.end());
	LPCWSTR swa = stempa.c_str();
	if (separatePlayers == 1)
		hFile = CreateFileW(swa, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	std::ostringstream pathb;
	pathb << ExePath() << "\\Output\\depthbg\\depthdata" << secCount <<","<< frameCount;
	std::string sb = pathb.str();
	std::wstring stempb = std::wstring(sb.begin(), sb.end());
	LPCWSTR swb = stempb.c_str();
	hFileb = CreateFileW(swb, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	ICoordinateMapper *mapper;
	sensor->get_CoordinateMapper(&mapper);
	int i = 0;
	int k = 0;


	mapper->MapDepthFrameToColorSpace(nBufferSize, pBuffer, nBufferSize, colorPoints);

	memcpy(players, pBodyBuffer, dwidth*dheight);
	for (int m = 0; m < dheight; m++){
		for (int n = 0; n < dwidth; n++)
		{
			USHORT depth = *pBuffer;
			BYTE player = *pBodyBuffer;

			//bool pl = isPlayer(i++, 9, n, m);
			++pBuffer;
			++pBodyBuffer;
			if (separatePlayers == 1 && player !=255){
				depths[j] = depth;
				depthsb[j++] = 0;
			}
			else{
				depths[j] = 0;
				depthsb[j++] = depth;
			}
			//DepthSpacePoint dsp;
			//dsp.X = n;
			//dsp.Y = m;
			ColorSpacePoint p = colorPoints[k++];
			//mapper->MapDepthPointToColorSpace(dsp, depth, &p);

			if (p.Y >= 0 && p.Y < height && p.X >= 0 && p.X < width){
				int idx = (int)((floor(p.Y) *(width * sizeof(RGBQUAD))) + ((floor(p.X) * sizeof(RGBQUAD))));
				colorBuffer[i++] = iniColorBuffer[idx++];
				colorBuffer[i++] = iniColorBuffer[idx++];
				colorBuffer[i++] = iniColorBuffer[idx++];
				colorBuffer[i++] = iniColorBuffer[idx++];
			}
			else{
				colorBuffer[i++] = 125;
				colorBuffer[i++] = 125;
				colorBuffer[i++] = 125;
				colorBuffer[i++] = 0xff;
			}
			DWORD dwBytesWritten = 0;
			if (j == BUFFERSIZE){
				j = 0;
				if (separatePlayers == 1)
					WriteFile(hFile, &depths, sizeof(depths), &dwBytesWritten, NULL);
				WriteFile(hFileb, &depthsb, sizeof(depthsb), &dwBytesWritten, NULL);
			}
		}
	}

	if (j != 0){
		DWORD size = j * sizeof(USHORT);
		DWORD dwBytesWritten = 0;
		if (separatePlayers == 1)
			WriteFile(hFile, &depths, size, &dwBytesWritten, NULL);
		WriteFile(hFileb, &depthsb, size, &dwBytesWritten, NULL);
	}
	if (separatePlayers == 1)
		CloseHandle(hFile);
	CloseHandle(hFileb);
	SaveBitmapToFile(static_cast<BYTE *>(colorBuffer), dwidth, dheight, 32, sw);
	SafeRelease(pDepthFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pColorFrame);
	return true;
}
std::chrono::high_resolution_clock::time_point lastCap;
std::chrono::high_resolution_clock::time_point lastSec;
bool dropped = false;

void drawImageData() {
	auto frequency = std::chrono::microseconds(33333);
	

	semaforo.lock();
	if (rec){
		semaforo.unlock();
		if (snap){
			bool didit = getandrecordData();
			if (didit){  
				semaforo.lock();
				rec = false;
				semaforo.unlock();
			}
		}
		else{
			auto  t1 = std::chrono::high_resolution_clock::now();
			if ((t1 - lastSec) >std::chrono::seconds(1)){
				secCount++;
				auto delay = t1 - lastSec - std::chrono::seconds(1);
				lastSec = std::chrono::high_resolution_clock::now() - delay;
			}
			auto delay = t1 - lastSec;
			std::chrono::microseconds ms = std::chrono::duration_cast<std::chrono::microseconds>(delay);
			frameCount = (int) std::floor(ms.count() / frequency.count());
			getandrecordDataToDisk();
				
		}
	}
	else{
		lastSec = std::chrono::high_resolution_clock::now();
		semaforo.unlock();
		getRgbImageData(rgbdata);
		getDepthImageData(depthdata);
	} 

	glBindTexture(GL_TEXTURE_2D, CtextureId);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, width, height, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)rgbdata);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glBegin(GL_QUADS);
	int theight1 =int(vheight * ((float)height / (float)width));
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f(0, 0, 0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f(vwidth, 0, 0);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f((GLfloat)vwidth, (GLfloat)theight1, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f(0, (GLfloat)theight1, 0.0f);
	glEnd();

	int theight = (int)(vheight * ((float)dheight / (float)dwidth));
	glBindTexture(GL_TEXTURE_2D, DtextureId);
	glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, dwidth, dheight, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)depthdata);
	glBegin(GL_QUADS);
	glTexCoord2f(0.0f, 0.0f);
	glVertex3f((GLfloat)vwidth, 0, 0);
	glTexCoord2f(1.0f, 0.0f);
	glVertex3f((GLfloat)vwidth + vwidth, 0, 0);
	glTexCoord2f(1.0f, 1.0f);
	glVertex3f((GLfloat)vwidth + vwidth, (GLfloat)theight, 0.0f);
	glTexCoord2f(0.0f, 1.0f);
	glVertex3f((GLfloat)vwidth, (GLfloat)theight, 0.0f);
	glEnd();

}

//for init;
int argcin;
char** argvin;

void kinectThreadTask() {
	if (!init(argcin, argvin)) return;
	if (!initKinect()) return;

	// Initialize textures
	glGenTextures(1, &CtextureId);
	glBindTexture(GL_TEXTURE_2D, CtextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, width, height, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)rgbdata);
	glBindTexture(GL_TEXTURE_2D, 0);

	glGenTextures(1, &DtextureId);
	glBindTexture(GL_TEXTURE_2D, DtextureId);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, dwidth, dheight, 0, GL_BGRA_EXT, GL_UNSIGNED_BYTE, (GLvoid*)depthdata);
	glBindTexture(GL_TEXTURE_2D, 0);

	// OpenGL setup
	glClearColor(0, 0, 0, 0);
	glClearDepth(1.0f);
	glEnable(GL_TEXTURE_2D);

	// Camera setup
	glViewport(0, 0, vwidth, vheight);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0, vwidth*2, vheight, 0, 1, -1);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();


	// Main loop
	execute();
	return ;
}

/*NETWORKING PART START */
class tcp_connection
	: public boost::enable_shared_from_this<tcp_connection>
{
public:
	typedef boost::shared_ptr<tcp_connection> pointer;
	string computerTimeSyncName;
	static pointer create(boost::asio::io_service& io_service)
	{
		return pointer(new tcp_connection(io_service));
	}

	tcp::socket& socket()
	{
		return socket_;
	}

	void loadConfig(){
		boost::property_tree::ptree pt;
		boost::property_tree::ini_parser::read_ini("config.ini", pt);
		computerTimeSyncName = pt.get<std::string>("timesync");
		separatePlayers = pt.get<int>("separatePlayers");
	}

	void syncComputerClocks(){
		stringstream ss1;
		ss1 << "net time \\\\" << computerTimeSyncName << " /set /yes";
		system(ss1.str().c_str());

	}


	void netSnapshot(bool acum){

		record(true,true,acum,false);
		char bufok[12];
		bool localrec = true;
		do{
			boost::this_thread::sleep(boost::posix_time::milliseconds(500));
			semaforo.lock();
			localrec = rec;
			semaforo.unlock();
		} while (localrec);

		boost::system::error_code error;
		size_t writtenBytes = boost::asio::write(socket_, boost::asio::buffer(depthsBuffer), error);
		std::cout << writtenBytes << "Bytes written\n";
		size_t len = socket_.read_some(boost::asio::buffer(bufok), error);
		std::cout << writtenBytes << "Received!\n";
		writtenBytes = boost::asio::write(socket_, boost::asio::buffer(colorBuffer), error);
		std::cout << writtenBytes << "Bytes written\n";
		len = socket_.read_some(boost::asio::buffer(bufok), error);
		printf("Transfer completed\n");
		socket_.close();
	}

	void netStartCap(){
		record(true,false,false,true);
	}

	void sendOk(){
		boost::system::error_code error;
		boost::asio::write(socket_, boost::asio::buffer("OK"), boost::asio::transfer_all(), error);
	}

	void netStopCap(){
		record(false,false,false,false);
	}

	void start()
	{	
		loadConfig();
		//syncComputerClocks();
		
		//Read request
		char bufReq[32];
		char bufTstp[256];
		boost::system::error_code error;
		size_t bytesRead =socket_.read_some(boost::asio::buffer(bufReq), error);
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
		std::cout<<"Request " << sReq << " now " << nowtime << " cap " << captime << "\n";
		unsigned long long sleeptime = captime - nowtime;
		sleeptime = sleeptime /1000;
		std::cout << "sleeping for " << sleeptime  << "ms \n";

		boost::this_thread::sleep(boost::posix_time::microseconds(sleeptime));
		
		if (sReq != "snap"){
			semaforo.lock();
			sensor->Close();
			semaforo.unlock();
		}

		if (sReq == "snap"){
			netSnapshot(false);
		}
		if (sReq == "snaplong"){
			netSnapshot(true);
		}
		if (sReq == "start"){
			netStartCap();
		}
		if (sReq == "stop"){
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
		: acceptor_(io_service, tcp::endpoint(tcp::v4(), 4141))
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
		boost::asio::io_service io_service;
		tcp_server server(io_service);
		io_service.run();
	}
	catch (std::exception& e)
	{
		std::cerr << e.what() << std::endl;
	}

	return ;
}
/*NETWORKING PART FINISH*/


int main(int argc, char* argv[]){
	argvin = argv;
	argcin = argc;
	boost::thread thread1(serverThreadTask); 
	boost::thread thread2(kinectThreadTask); 
	thread1.join();
	thread2.join();
	return 0;
}