/*
 * types.hpp
 *
 *  Created on: 2016年12月19日
 *      Author: work
 */

#ifndef INCLUDE_DM_OS_COM_TYPES_HPP_
#define INCLUDE_DM_OS_COM_TYPES_HPP_

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/smart_ptr.hpp>

namespace dm{
namespace os{
namespace com{

typedef boost::asio::io_service ios_t;
typedef boost::system::error_code error_t;

}
}
}

#endif /* INCLUDE_DM_OS_COM_TYPES_HPP_ */
