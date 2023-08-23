.PHONY: run clean test
VENV = venv
PYTHON = ./$(VENV)/Scripts/python

run: $(VENV)/Scripts/activate
	$(PYTHON) src/serial_interface.py


$(VENV)/Scripts/activate: requirements.txt
	python -m venv $(VENV) --upgrade-deps
	$(PYTHON) -m pip install -r requirements.txt
	$(PYTHON) -m pip list


clean:
	rm -rf __pycache__
	rm -rf $(VENV)


test: $(VENV)/Scripts/activate
	$(PYTHON) tests/test.py -v