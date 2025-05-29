/*
 * timer.hpp
 *
 *  Created on: 2016年12月19日
 *      Author: work
 */

#ifndef _DM_OS_ASCOM_TIMER_HPP_
#define _DM_OS_ASCOM_TIMER_HPP_

#include <dm/os/com/timer.hpp>
#include <dm/os/ascom/types.hpp>

namespace dm{
namespace os{
namespace ascom{

class CTimer:public com::CTimer{
public:
	CTimer( com::CTimer::ios_t& ios ):com::CTimer(ios){

	}
	~CTimer(){

	}

	boost::signals2::signal<void()> sig_timeout;

protected:
	void on_timeout(){
		sig_timeout();
	}
};

}
}
}



#endif /* INCLUDE_DM_OS_ASCOM_TIMER_HPP_ */
