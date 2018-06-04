.PHONY: all
all: GUI Math Simulator Renderer Optimization VirtualPhysics

.PHONY: GUI
GUI:
	make --directory="GUI/" --file=GUI.makefile

.PHONY: Math
Math:
	make --directory="Math/" --file=Math.makefile

.PHONY: Mesh
Mesh:
	make --directory="Mesh/" --file=Mesh.makefile

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
	rm -rf Math/build
	rm -rf GUI/build
	rm -rf Renderer/build
	rm -rf Simulator/build
	rm -rf Optimization/build
	rm -rf VirtualPhysics/build
