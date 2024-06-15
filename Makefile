.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "make test                                      Run the host C++ tests"
	@echo "python -m tools.build_firmware build <board> <controller>  Build a firmware image"
	@echo "Boards: mega, bluepill. Controllers: pid, lead, lqr, mpc."

.PHONY: test
test:
	$(MAKE) -C tests run
