PY_VER = $(shell /usr/bin/env python3 -c "import sys; print(str(sys.version_info[0])+'.'+str(sys.version_info[1]))")
#PY_SITE = $(shell /usr/bin/env python3 -m site --user-site)
PY_SITE = $(VIRTUAL_ENV)/lib/python$(PY_VER)/site-packages

.NOTPARALLEL:

default: compile

compile:
	$(MAKE) -C build

docs:
	cd rai-docs && sphinx-build doc ../html

info:
	@echo "PY_VER:" $(PY_VER)
	@echo "PY_SITE:" $(PY_SITE)

docs-clean:
	rm -Rf html

local-install:
	ln -f -s _make/CMakeLists-ubuntu.txt CMakeLists.txt
	-rm -f ${HOME}/.local/lib/*rai*
	cmake . -B build -DPY_VERSION=$(PY_VER)
	$(MAKE) -C build _robotic docstrings install -j $(shell nproc --ignore 2)
	cp build/_robotic.pyi $(PY_SITE)/robotic

local-clean:
	-rm -Rf $(PY_SITE)/robotic
	-rm -Rf $(PY_SITE)/robotic-*
	-rm -f ${HOME}/.local/lib/*rai*
	-rm -f ${HOME}/.local/bin/*ry*
	-rm -Rf dist src/robotic.egg-info build/lib build/bdist.*

wheels:
	$(eval id = $(shell _make/run-docker.sh rai-manylinux -d))
	@echo "started docker " ${id}
	docker exec -it ${id} /bin/bash -C local/_make/build-wheels.sh
	docker stop ${id}
#	_make/run-docker.sh local/_make/build-wheels.sh

wheels-upload:
	twine upload dist/*.whl

wheels-install:
	python$(PY_VER) -m pip install dist/robotic-*cp312*.whl --force-reinstall

test-buildFromScratch:
	$(eval id = $(shell _make/run-docker.sh ubuntu -d))
	@echo "started docker " ${id}
	docker exec -it ${id} /bin/bash -C /root/local/_make/test-build.sh
	docker stop ${id}

test:
	cd ${HOME} && python3 -c 'import robotic as ry; print("ry version:", ry.__version__, ry.compiled());'

test2:
	cd ${HOME} && python3 -c 'import robotic as ry; ry.test.RndScene()'

test3:
	ry-view $(PY_SITE)/robotic/rai-robotModels/scenarios/pandasTable.g

test-tutorials:
	$(MAKE) -j1 -C rai-tutorials run

pull:
	cd rai && git pull
	cd rai-robotModels && git pull
	cd rai-docs && git pull
	cd rai-tutorials && git pull
	cd botop && git pull

docker-clean:
	$(shell docker container kill "$(docker container ls -q)")
	docker system prune

