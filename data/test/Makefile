
PEM = insecure.pem hogehoge.pem
KEY = hogehoge.key
PUB = $(PEM:.pem=.pub)

%.pub: %.pem
	openssl rsa -in $< -pubout > $@

all: $(PUB)

clean:
	$(RM) *.pub *.old *.bak *~

purge: clean
	$(RM) $(PEM) $(KEY)

insecure.pem:
	openssl genrsa -out $@ 2048

hogehoge.key:
	openssl genrsa -des3 -passout pass:hogehoge -out $@ 2048

hogehoge.pem: hogehoge.key
	openssl rsa -passin pass:hogehoge -in $< -out $@ -outform PEM
