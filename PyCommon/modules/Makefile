.PHONY: all
all: Simulator Renderer Optimization VirtualPhysics

.PHONY: Renderer
Renderer:
	make --directory="Renderer/" --file=Renderer.makefile

.PHONY: Simulator
Simulator:
	make --directory="Simulator/" --file=Simulator.makefile

.PHONY: Optimization
Optimization:
	make --directory="Optimization/" --file=Optimization.makefile

.PHONY: VirtualPhysics
VirtualPhysics:
	make --directory="VirtualPhysics/" --file=VirtualPhysics.makefile

.PHONY: clean
clean:
	rm -rf Renderer/build
	rm -rf Simulator/build
	rm -rf Optimization/build
	rm -rf VirtualPhysics/build
