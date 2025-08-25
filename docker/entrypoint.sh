#!/bin/bash

service ssh restart

# Execute the main command passed to the container (e.g., from CMD or docker run)
exec "$@"