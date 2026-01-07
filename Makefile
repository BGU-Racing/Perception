# --- Config ---
IMAGE_NAME := ros2-jazzy-dev-image
CONTAINER_NAME := ros2-jazzy-dev-container
WORKSPACE_DIR := $(CURDIR)/..

.PHONY: build run attach stop clean help

# --- Build Docker Image ---
build:
	@echo "🚀 Building Docker image: $(IMAGE_NAME)"
	docker build -t $(IMAGE_NAME) -f deployment/Dockerfile .

# --- Run Container ---
run: build
	@echo "🏃 Starting container: $(CONTAINER_NAME)"
	@if [ "$$(docker ps -aq -f name=^$(CONTAINER_NAME)$$)" ]; then \
		echo "🔄 Reusing existing container..."; \
		docker start -ai $(CONTAINER_NAME); \
	else \
		echo "✨ Creating new container..."; \
		docker run -it --name $(CONTAINER_NAME) \
			-v $(WORKSPACE_DIR):/workspace \
			$(IMAGE_NAME); \
	fi

# --- Open a NEW shell in the container (recommended) ---
attach:
	@echo "🔗 Opening shell in container: $(CONTAINER_NAME)"
	docker exec -it $(CONTAINER_NAME) /bin/bash

# --- Stop Container ---
stop:
	@echo "🛑 Stopping container..."
	docker stop $(CONTAINER_NAME) || true

# --- Delete Container ---
clean:
	@echo "🗑 Removing container..."
	docker rm -f $(CONTAINER_NAME) || true

help:
	@echo "Available commands:"
	@echo "  make build    - Build Docker image"
	@echo "  make run      - Run (or start) Docker container"
	@echo "  make attach   - Open a new shell in the container"
	@echo "  make stop     - Stop container"
	@echo "  make clean    - Remove container"
