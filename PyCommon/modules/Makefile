.PHONY: all
all: Renderer Simulator

.PHONY: Renderer
Renderer:
	make --directory="Renderer/" --file=Renderer.makefile

.PHONY: Simulator
Simulator:
	make --directory="Simulator/" --file=Simulator.makefile

.PHONY: clean
clean:
	rm -rf Renderer/build
	rm -rf Simulator/build
