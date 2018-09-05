#pragma once
const int dwidth = 512;
const int dheight = 424;
const int width = 1920;
const int height = 1080;
const int vwidth = 1280;
const int vheight = 720;
const int BUFFERSIZE = 10240;

void drawKinectData();
void drawImageData();
void record(bool start, bool single,bool acum,bool reset);
HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR path);

template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}