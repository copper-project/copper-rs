FROM ubuntu:22.04

# Install necessary system apt libs
RUN apt update \
	&& apt install -y --no-install-recommends \ 
	git \
	curl \ 
	build-essential \
	pkg-config \
	libudev-dev

# Install cargo
RUN curl https://sh.rustup.rs -sSf | bash -s -- -y

# Set up entry environment
ENV PATH="/root/.cargo/bin:${PATH}"
CMD ["/bin/bash"]
WORKDIR /home/copper-rs