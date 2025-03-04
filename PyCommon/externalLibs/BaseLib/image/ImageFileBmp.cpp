#include "stdafx.h"
#include "stdimage.h"
#include "Image.h"
#include "../baselib/utility/fileex.h"

#define DIB_HEADER_MARKER   ((WORD) ('M' << 8) | 'B')
/////////////////////////////////////////////////////////////////////////////
// CCImage load/save bmp

BOOL CImage::SaveBMP(LPCTSTR lpszFileName)
{
	CFileEx file;
	//CFileExException fe;
	BITMAPFILEHEADER bmfHdr;
	LPBITMAPINFOHEADER lpBI;
	DWORD dwDIBSize;

	// 쓰기 모드로 파일 열기
	if (!file.Open(lpszFileName, CREATE_ALWAYS, GENERIC_WRITE))
		return FALSE;

	// 메모리 핸들이 유효한지 확인
	if (m_hImage == NULL) return FALSE;

	// 메모리 고정
	lpBI = (LPBITMAPINFOHEADER)::GlobalLock((HGLOBAL)m_hImage);
	if (lpBI == NULL) return FALSE;

	// 비트맵 파일 헤더 정보를 설정
	bmfHdr.bfType = DIB_HEADER_MARKER;  // "BM"
	dwDIBSize = *(LPDWORD)lpBI + ::PaletteSize((LPSTR)lpBI);
	if((lpBI->biCompression==BI_RLE8) || (lpBI->biCompression==BI_RLE4))
		dwDIBSize += lpBI->biSizeImage;
	else 
	{
		DWORD dwBmBitsSize;  // Size of Bitmap Bits only
		dwBmBitsSize = WIDTHBYTES((lpBI->biWidth)*((DWORD)lpBI->biBitCount)) * lpBI->biHeight;
		dwDIBSize += dwBmBitsSize;
		lpBI->biSizeImage = dwBmBitsSize;
	}

	bmfHdr.bfSize = dwDIBSize + sizeof(BITMAPFILEHEADER);
	bmfHdr.bfReserved1 = 0;
	bmfHdr.bfReserved2 = 0;
	bmfHdr.bfOffBits=(DWORD)sizeof(BITMAPFILEHEADER)+lpBI->biSize + PaletteSize((LPSTR)lpBI);
	try
	{
		// 비트맵 파일 헤더를 파일에 쓰기
		file.Write((BYTE*)&bmfHdr, sizeof(BITMAPFILEHEADER));
		// 나머지 데이터를 파일에 쓰기
		//file.WriteHuge(lpBI, dwDIBSize);
		file.Write((BYTE*)lpBI, dwDIBSize);
	}
	catch (CFileExException e)
	{
		::GlobalUnlock((HGLOBAL) m_hImage);
		//throwTHROW_LAST();
	}
	//END_CATCH

	// 메모리 풀어줌
	::GlobalUnlock((HGLOBAL) m_hImage);
	return TRUE;
}


BOOL CImage::LoadBMP(LPCTSTR lpszFileName)
{
	CFileEx file;
	LPSTR pDIB;
	DWORD dwBitsSize;
	BITMAPFILEHEADER bmfHeader;

	// 읽기 모드로 파일 열기

	if(!file.Open(lpszFileName, OPEN_EXISTING, GENERIC_READ))
		return FALSE;

	// 파일의 길이를 구함
	dwBitsSize = file.GetFileSize();

	// 파일 헤더 읽기
	if(file.Read((BYTE*)&bmfHeader, sizeof(bmfHeader))!=sizeof(bmfHeader))
		return FALSE;

	// BMP 파일임을 나타내는 "BM" 마커가 있는지 확인
	if (bmfHeader.bfType != DIB_HEADER_MARKER)
		return FALSE;

	// 메모리 할당
	if((m_hImage = (HDIB)::GlobalAlloc(GMEM_MOVEABLE | GMEM_ZEROINIT, dwBitsSize)) == NULL) return FALSE;

	// 메모리 고정
	pDIB = (LPSTR) ::GlobalLock((HGLOBAL) m_hImage);

	// 파일 읽기
	if (file.Read((BYTE*)pDIB, dwBitsSize - sizeof(BITMAPFILEHEADER)) != dwBitsSize - sizeof(BITMAPFILEHEADER) ) 
	{
		::GlobalUnlock((HGLOBAL) m_hImage);
		::GlobalFree((HGLOBAL) m_hImage);
		return FALSE;
	}

	// 메모리 풀어줌
	::GlobalUnlock((HGLOBAL) m_hImage);

	// DIB 초기화
	InitDIB(false);

	return TRUE;
}
