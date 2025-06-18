# Makefile for ESC ECU Simulation

CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -O2 -g
LDFLAGS = -lm -lpthread
TARGET = esc_ecu
SOURCES = main.c esc_ecu.c sensor_interface.c digital_filters.c sensor_plausibility.c esc_control_algorithm.c brake_actuation.c engine_torque_interface.c watchdog_supervision.c dtc_management.c can_bus_interface.c post_system.c esc_logging.c
OBJECTS = $(SOURCES:.c=.o)
HEADERS = esc_types.h esc_ecu.h sensor_interface.h sensor_plausibility.h esc_control_algorithm.h brake_actuation.h engine_torque_interface.h watchdog_supervision.h dtc_management.h can_bus_interface.h post_system.h esc_logging.h

# Demo targets
DEMO_TARGETS = brake_demo engine_demo watchdog_demo dtc_demo can_demo post_demo logging_demo

# Default target
all: $(TARGET) $(DEMO_TARGETS)

# Build the executable
$(TARGET): $(OBJECTS)
	$(CC) $(OBJECTS) -o $(TARGET) $(LDFLAGS)
	@echo "Build complete: $(TARGET)"

# Compile source files
%.o: %.c $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@

# Build demo executables
brake_demo: brake_demo.c brake_actuation.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

engine_demo: engine_demo.c engine_torque_interface.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

watchdog_demo: watchdog_demo.c watchdog_supervision.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

dtc_demo: dtc_demo.c dtc_management.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

can_demo: can_demo.c can_bus_interface.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

post_demo: post_demo.c post_system.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

logging_demo: logging_demo.c esc_logging.c sensor_interface.c digital_filters.c
	$(CC) $(CFLAGS) $^ -o $@ $(LDFLAGS)

# Clean build artifacts
clean:
	rm -f $(OBJECTS) $(TARGET) $(DEMO_TARGETS)
	@echo "Clean complete"

# Run the simulation
run: $(TARGET)
	./$(TARGET)

# Run with verbose output
run-verbose: $(TARGET)
	./$(TARGET) -v

# Install (copy to /usr/local/bin)
install: $(TARGET)
	sudo cp $(TARGET) /usr/local/bin/

# Uninstall
uninstall:
	sudo rm -f /usr/local/bin/$(TARGET)

# Show help
help:
	@echo "Available targets:"
	@echo "  all         - Build the ESC ECU simulation and all demos (default)"
	@echo "  clean       - Remove build artifacts"
	@echo "  run         - Build and run the simulation"
	@echo "  run-verbose - Build and run with verbose output"
	@echo "  brake_demo  - Build and run brake actuation demo"
	@echo "  engine_demo - Build and run engine torque demo"
	@echo "  watchdog_demo - Build and run watchdog supervision demo"
	@echo "  dtc_demo    - Build and run DTC management demo"
	@echo "  can_demo    - Build and run CAN bus interface demo"
	@echo "  post_demo   - Build and run POST system demo"
	@echo "  logging_demo - Build and run logging system demo"
	@echo "  install     - Install to /usr/local/bin"
	@echo "  uninstall   - Remove from /usr/local/bin"
	@echo "  help        - Show this help message"

.PHONY: all clean run run-verbose install uninstall help