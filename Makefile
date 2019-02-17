.PHONY: all
all:
	pio run

.PHONY: upload
upload:
	pio run -t $@

.PHONY: clean
clean:
	pio run --target clean

