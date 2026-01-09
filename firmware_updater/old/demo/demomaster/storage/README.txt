Place the binaries you want packaged into the SPIFFS partition here. The build now copies
`demo/demoslave/artifacts/bye.bin` into this folder automatically (before the SPIFFS image
is generated) so the master always serves the latest slave artifact. If you want to stage
other payloads, drop them in this directory before running `idf.py build`.
