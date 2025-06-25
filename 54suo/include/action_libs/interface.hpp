#ifndef ZYZN_PLUGIN_INTERFACE_HPP
#define ZYZN_PLUGIN_INTERFACE_HPP
#include <memory>
#include <boost/dll/alias.hpp>

namespace zyzn{
    namespace plugin{
        class BasePlugin;
        std::shared_ptr<BasePlugin> create_plugin();
    }
}

BOOST_DLL_ALIAS(
    zyzn::plugin::create_plugin,                        // <-- this function is exported with...
    create_plugin                                       // <-- ...this alias name
)


#endif