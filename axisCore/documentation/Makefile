HTMLFLAGS = --book --duplex --no-numbered --format pdf14 --title --size letter

%.pdf: %.html
	-htmldoc $(HTMLFLAGS) --outfile $@ $<

all: doxygen install

doxygen:
	doxygen

install:
	$(CP) -f *.html /home/epics/web_software/epics
	$(CP) -f *.png /home/epics/web_software/epics
	$(CP) -rfp motorDoxygenHTML  /home/epics/web_software/epics/

clean:
	$(RM) -rf motorDoxygenHTML/*


realclean: clean
