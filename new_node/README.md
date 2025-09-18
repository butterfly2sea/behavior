```
behavior_node/
├── CMakeLists.txt
├── package.xml
├── include/
│   └── behavior_node/
│       ├── core/
│       │   ├── behavior_executor.hpp
│       │   ├── behavior_node_main.hpp
│       │   ├── message_queue.hpp
│       │   └── types.hpp
│       ├── data/
│       │   ├── data_cache.hpp
│       │   ├── mission_context.hpp
│       │   ├── ros_communication_manager.hpp
│       │   ├── ros_interface_definitions.hpp
│       │   └── base_enum.hpp
│       ├── base/
│       │   ├── base_nodes.hpp
│       │   └── node_dependencies.hpp
│       ├── actions/
│       │   ├── flight_actions.hpp
│       │   ├── navigation_actions.hpp
│       │   ├── search_actions.hpp
│       │   ├── control_actions.hpp
│       │   └── formation_actions.hpp
│       ├── conditions/
│       │   ├── flight_conditions.hpp
│       │   ├── navigation_conditions.hpp
│       │   ├── search_conditions.hpp
│       │   └── system_conditions.hpp
│       └── utils/
│           ├── task_validator.hpp
│           └── coordinate_converter.hpp
├── src/
│   ├── core/
│   │   ├── behavior_executor.cpp
│   │   ├── behavior_node_main.cpp
│   │   └── message_queue.cpp
│   ├── data/
│   │   ├── data_cache.cpp
│   │   ├── mission_context.cpp
│   │   ├── ros_communication_manager.cpp
│   │   └── ros_interface_definitions.cpp
│   ├── actions/
│   │   ├── flight_actions.cpp
│   │   ├── navigation_actions.cpp
│   │   ├── search_actions.cpp
│   │   ├── control_actions.cpp
│   │   └── formation_actions.cpp
│   ├── conditions/
│   │   ├── flight_conditions.cpp
│   │   ├── navigation_conditions.cpp
│   │   ├── search_conditions.cpp
│   │   └── system_conditions.cpp
│   ├── utils/
│   │   ├── task_validator.cpp
│   │   └── coordinate_converter.cpp
│   └── main.cpp
├── tree/
│   ├── main_behavior_tree.xml
│   └── config/
│       ├── node_config.yaml
│       └── tree_config.yaml
└── launch/
    └── behavior_node.launch.py
```