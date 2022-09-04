#!/bin/bash

# source .devcontainer/best_registry.bash

# Map port 8080 of plantuml-server to 32753 on host:
docker start plantuml_server || docker run --name plantuml_server -d -p 32753:8080 plantuml/plantuml-server:jetty

# docker login registry.dorabot.com

# Map port 8888 of mangrove_planner_integration to 8889 on host to not conflict with the UI:
docker start mangrove_planner || docker run --name mangrove_planner -d -p 8889:8888 registry.dorabot.com/aiot/mangrove/mangrove_planner_integration:v1.3.10-beta
