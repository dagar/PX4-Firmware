/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

template< class T >
class LoggingTask : public T
{
public:
	void Execute()
	{
		std::cout << "LOG: The task is starting - " << GetName().c_str() << std::endl;
		T::Execute();
		std::cout << "LOG: The task has completed - " << GetName().c_str() << std::endl;
	}
};

template<class T>
class TimingTask : public T
{
	Timer timer_;
public:
	void Execute()
	{
		timer_.Reset();
		T::Execute();
		double t = timer_.GetElapsedTimeSecs();
		std::cout << "Task Duration: " << t << " seconds" << std::endl;
	}
};

class MyTask
{
public:
	void Execute()
	{
		std::cout << "...This is where the task is executed..." << std::endl;
	}

	std::string GetName()
	{
		return "My task name";
	}
};

class PX4Driver
{
public:
	virtual bool Start() = 0;
	virtual bool Stop() = 0;
	virtual bool Status() = 0;
	virtual bool PrintStatus() = 0;
}

template<typename Interface, typename SchdedulePolicy>
class MPU6000 : public PX4Driver, private Interface
{
public:
	virtual void measure()
	{
		RegisterRead();
	}

private:
	using Interface::RegisterRead;
};

template<typename T>
class IntervalScheduled : public T, public px4::ScheduledWorkItem
{
	void Run() override;
};



void main()
{


	InvervalScheduled<MPU6000<SPI>


}
