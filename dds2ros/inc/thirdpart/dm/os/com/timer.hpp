/*
 * timer.hpp
 *
 *  Created on: 2016年12月19日
 *      Author: work
 */

#ifndef _DM_OS_COM_TIMER_HPP_
#define _DM_OS_COM_TIMER_HPP_

#include <dm/types.hpp>
#include <dm/os/com/types.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

namespace dm{
namespace os{
namespace com{

class  CTimer{
public:
	typedef boost::asio::io_service ios_t;
	typedef boost::system::error_code& error_t;
	typedef boost::asio::deadline_timer dev_t;
	typedef boost::posix_time::time_duration time_t;
	typedef dm::uint32 sec_t;
	typedef dm::uint32 msec_t;
	typedef dm::uint64 nsec_t;

public:
	CTimer( ios_t& ios ):m_dev(ios),m_d(),m_auto(false){}
	virtual ~CTimer(){}

	inline void start( time_t interval ){
		start(false,interval);
	}

	inline void start(){
		start(false);
	}

	inline void startAuto( time_t interval ){
		start(true,interval);
	}

	inline void startAuto(){
		start(true);
	}

	void start( bool a ){
		m_auto = a;
		start_timer();
	}
	void start( bool a,time_t interval ){
		m_auto = a;
		m_d = interval;
		start_timer();
	}


	inline void start_seconds( sec_t sec ){
		start( boost::posix_time::seconds(sec));
	}

	inline void start_seconds( bool a,sec_t sec ){
		start(a,boost::posix_time::seconds(sec));
	}

	// 毫秒
	inline void start_nseconds( nsec_t nsec ){
		start( boost::posix_time::time_duration(0,0,0,nsec*1000l) );
	}

	inline void start_nseconds( bool a,nsec_t nsec ){
		start( a,boost::posix_time::time_duration(0,0,0,nsec*1000l) );
	}

	inline void cancel(){
		m_dev.cancel();
	}

protected:
	void async_wait( const boost::system::error_code& error ){
		if(error)
			on_timercancel();
		else{
			on_timeout();
			if(m_auto)
				start_timer();
		}
	}

	void start_timer(){
		m_dev.expires_from_now(m_d);
		m_dev.async_wait(boost::bind(&CTimer::async_wait,this,boost::asio::placeholders::error));
	}

	virtual void on_timeout(){}
	virtual void on_timercancel(){}

protected:
	dev_t m_dev;
	time_t m_d;
	bool m_auto;
};

}
}
}



#endif /* INCLUDE_DM_OS_COM_TIMER_HPP_ */
