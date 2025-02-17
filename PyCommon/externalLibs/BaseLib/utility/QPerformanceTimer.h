/********************************************************************************
	Copyright (C) 2004 Sjaak Priester	

	This is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	This file is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Tinter; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
********************************************************************************/

// QPerformanceTimer
// Class to measure performance time
//
// Usage: simply define a QPerformanceTimer variable in a C++ block.
// At construction, it measures the start time.
// At destruction, it measures the stop time. It calculates the difference
// and puts the result (in milliseconds) in the associated int.
// In other words: this class measures and records its own lifetime.
// If the output is set to -1, it's an indication of overflow, but this
// will only occur after more than three weeks (!).
//
// Example:
//
//
//	SomeFunction()
//	{
//		... some code...
//
//		int elapsedTime;
//		{	// start of the block we want to monitor
//			QPerformanceTimer(elapsedTime);
//
//			... some lengthy process...
//
//		}	// end of block, elapsed time is recorded in elapsedTime
//
//		printf("Time = %d milliseconds", elapsedTime);
//	}
//
// Version 1.0 (C) 2004, Sjaak Priester, Amsterdam.
// mailto:sjaak@sjaakpriester.nl

#pragma once


class QPerformanceTimer
{
public:
	QPerformanceTimer(int& MilliSeconds)
		: m_Output(MilliSeconds)
	{
		::QueryPerformanceCounter(&m_Start);
	}

	~QPerformanceTimer(void)
	{
		LARGE_INTEGER stop;
		LARGE_INTEGER freq;
		::QueryPerformanceCounter(&stop);
		::QueryPerformanceFrequency(&freq);

		stop.QuadPart -= m_Start.QuadPart;
		stop.QuadPart *= 1000;
		stop.QuadPart /= freq.QuadPart;

		if (stop.HighPart != 0) m_Output = -1;
		else m_Output = stop.LowPart;
	}
protected:
	LARGE_INTEGER m_Start;
	int& m_Output;
};

#define BEGIN_TIMER(x)	QPerformanceTimer2 x
#define END_TIMER(x)	printf("%s Time= %d milliseconds\n", #x, x.stop())

// taesoo modified
class QPerformanceTimer2
{
public:
	QPerformanceTimer2()
	{
		start();
	}

	~QPerformanceTimer2(void){}

	void start()
	{
		::QueryPerformanceCounter(&m_Start);
	}

	int stop()
	{
		LARGE_INTEGER stop;
		LARGE_INTEGER freq;
		::QueryPerformanceCounter(&stop);
		::QueryPerformanceFrequency(&freq);

		stop.QuadPart -= m_Start.QuadPart;
		stop.QuadPart *= 1000;
		stop.QuadPart /= freq.QuadPart;

		if (stop.HighPart != 0) return -1;
		return stop.LowPart;
	}

protected:
	LARGE_INTEGER m_Start;
};