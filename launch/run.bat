echo "Starting seoul-cits-autonomous-relay..."
@echo off
start "seoul-cits-autonomous-relay" java -jar seoul-cits-autonomous-relay-1.0.jar  --spring.config.location=.\conf\application.yml
