# Makefile for Pybricks Pico W / Pico 2 W
# Builds both targets from unified source

# Configuration
PICO_SDK_PATH ?= $(HOME)/pico-sdk
BUILD_BASE := build

# Export for CMake
export PICO_SDK_PATH

# Detect number of CPU cores
NPROC := $(shell nproc 2>/dev/null || sysctl -n hw.ncpu 2>/dev/null || echo 4)
MAKEFLAGS += -j$(NPROC)

# Colors
YELLOW := \033[1;33m
GREEN := \033[0;32m
RED := \033[0;31m
NC := \033[0m

.PHONY: all picow pico2w clean help

all: picow pico2w

picow:
	@echo "$(YELLOW)Building Pico W (RP2040, 4 motors)...$(NC)"
	@mkdir -p $(BUILD_BASE)/picow
	@cd $(BUILD_BASE)/picow && cmake -DTARGET_BOARD=picow ../.. && $(MAKE) --no-print-directory
	@if [ -f $(BUILD_BASE)/picow/pybricks_picow.uf2 ]; then \
		echo "$(GREEN)✓ Pico W: $(BUILD_BASE)/picow/pybricks_picow.uf2$(NC)"; \
	else \
		echo "$(RED)✗ Pico W build failed$(NC)"; exit 1; \
	fi

pico2w:
	@echo "$(YELLOW)Building Pico 2 W (RP2350B, 12 motors)...$(NC)"
	@mkdir -p $(BUILD_BASE)/pico2w
	@cd $(BUILD_BASE)/pico2w && cmake -DTARGET_BOARD=pico2w ../.. && $(MAKE) --no-print-directory
	@if [ -f $(BUILD_BASE)/pico2w/pybricks_pico2w.uf2 ]; then \
		echo "$(GREEN)✓ Pico 2 W: $(BUILD_BASE)/pico2w/pybricks_pico2w.uf2$(NC)"; \
	else \
		echo "$(RED)✗ Pico 2 W build failed$(NC)"; exit 1; \
	fi

clean:
	@echo "$(YELLOW)Cleaning build directories...$(NC)"
	@rm -rf $(BUILD_BASE)
	@echo "$(GREEN)✓ Clean complete$(NC)"

clean-picow:
	@rm -rf $(BUILD_BASE)/picow

clean-pico2w:
	@rm -rf $(BUILD_BASE)/pico2w

help:
	@echo "Pybricks Pico Build System"
	@echo ""
	@echo "Targets:"
	@echo "  make              - Build both Pico W and Pico 2 W (default)"
	@echo "  make picow        - Build Pico W only (4 motors)"
	@echo "  make pico2w       - Build Pico 2 W only (12 motors)"
	@echo "  make clean        - Remove all build directories"
	@echo ""
	@echo "Environment:"
	@echo "PICO_SDK_PATH=$(PICO_SDK_PATH)"