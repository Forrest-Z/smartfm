all:
	mkdir -p classes
	javac -d classes -classpath classes $(shell find src -name "*.java")
	jar cfe FMUI.jar fmui.FMUI -C classes fmui

clean:
	rm -R classes/* *.jar
