.PHONY: run clean test
VENV = venv
PYTHON = ./$(VENV)/Scripts/python
PYTHON_VER := $(shell python3 --version 2>&1)

run: $(VENV)/Scripts/activate
	$(PYTHON) src/serial_interface.py


$(VENV)/Scripts/activate: requirements.txt
ifeq ('$(PYTHON_VER)','')		
	python -m venv $(VENV) --upgrade-deps
	$(PYTHON) -m pip install logging
else
	python3 -m venv $(VENV) 
	$(eval PYTHON=./$(VENV)/bin/python3)
endif
	$(PYTHON) -m pip install -r requirements.txt

clean:
	rm -rf __pycache__
	rm -rf $(VENV)


test: $(VENV)/Scripts/activate
	$(PYTHON) tests/test.py -v

sim: $(VENV)/Scripts/activate
	$(PYTHON) tests/sim.py
