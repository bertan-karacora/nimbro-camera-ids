FROM amd64/ros:humble-perception AS base-amd64



# ========================================
FROM arm64v8/ros:humble-perception AS base-arm64



# ========================================
FROM "base-$TARGETARCH" AS base

ARG DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NOWARNINGS="yes"

SHELL ["/bin/bash", "-c"]

# Install apt-utils
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    apt-utils \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /var/lib/apt/lists/*

# Install essentials
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    build-essential \
    ca-certificates \
    curl \
    git \
    lsb-release \
    nano \
    software-properties-common \
    tmux \
    unzip \
    wget \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /var/lib/apt/lists/*



# ========================================
FROM base AS system

# Install dependencies
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    libatomic1 \
    libqt5core5a \
    libqt5gui5 \
    libqt5multimedia5 \
    libqt5quick5 \
    libqt5widgets5 \
    libusb-1.0-0 \
    python3 \
    python3-pip \
    python3-venv \
    python3-numpy \
    qml-module-qtquick2 \
    qml-module-qtquick-window2 \
    qml-module-qtquick-dialogs \
    qml-module-qtquick-controls \
    qml-module-qtquick-layouts \
    qml-module-qt-labs-folderlistmodel \
    qml-module-qt-labs-settings \
    qtbase5-dev \
    qtdeclarative5-dev \
    udev \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /var/lib/apt/lists/*

# Install IDS peak
# Assume package is in /resources
ARG TARGETARCH
COPY resources /tmp/resources
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    /tmp/resources/ids-peak_*_"$TARGETARCH".deb \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /tmp/* /var/lib/apt/lists/*

# Alternative: Install IDS peak
# RUN wget \
#     --http-user=$IDS_USERNAME \
#     --http-password=$IDS_PASSWORD \
#     -O- https://repo.ids-imaging.com/ids.pgp |\
#     gpg --dearmor |\
#     tee /usr/local/share/keyrings/ids-archive-keyring.gpg >\
#     /dev/null
# RUN touch /etc/apt/auth.conf.d/ids-peak-repo.conf
# RUN echo "machine repo.ids-imaging.com/ubuntu login $IDS_USERNAME password $IDS_PASSWORD" |\
#     tee /etc/apt/auth.conf.d/ids-peak-repo.conf >\
#     /dev/null
# RUN touch /etc/apt/sources.list.d/ids-peak-repo.list
# RUN echo "deb [signed-by=/usr/local/share/keyrings/ids-archive-keyring.gpg] https://repo.ids-imaging.com/ubuntu jammy main" |\
#     tee /etc/apt/sources.list.d/ids-peak-repo.list >\
#     /dev/null
# RUN apt-get update -qq
# RUN apt-get install -y -qq --no-install-recommends \
#     ids-peak \
#     && \
#     apt-get autoremove -y -qq && \
#     apt-get clean -qq && \
#     rm -rf /tmp/* /var/tmp/* /var/lib/apt/lists/*

# Install tools
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    gettext-base \
    ros-humble-rmw-cyclonedds-cpp \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /var/lib/apt/lists/*




# ========================================
FROM system AS python

# Install python
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    python-is-python3 \
    python3 \
    python3-pip \
    python3-venv \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /var/lib/apt/lists/*

# Install python version explicitly
RUN add-apt-repository -y ppa:deadsnakes/ppa
RUN apt-get update -qq && \
    apt-get install -y -qq --no-install-recommends \
    python3.10 \
    python3.10-venv \
    && \
    apt-get autoremove -y -qq && \
    apt-get clean -qq && \
    rm -rf /var/lib/apt/lists/*

ENV PYTHONUNBUFFERED=1 \
    PYTHONWARNINGS=ignore:::setuptools.command.install,ignore:::setuptools.command.easy_install,ignore:::setuptools.command.develop,ignore:::setuptools.dist,ignore:::pkg_resources \
    PIP_DISABLE_PIP_VERSION_CHECK=on \
    PIP_ROOT_USER_ACTION=ignore

# Setup venv
RUN python3.10 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH" \
    PYTHONPATH="/opt/venv/lib/python3.10/site-packages:$PYTHONPATH"
RUN pip install --upgrade pip

# Install dependencies
COPY requirements.txt /tmp/requirements.txt
RUN pip install --no-cache-dir --requirement /tmp/requirements.txt && \
    rm -rf /tmp/*



# ========================================
FROM python AS camera_ids

# Add code
ARG USERNAME_GITLAB \
    TOKEN_GITLAB
RUN git clone "https://$USERNAME_GITLAB:$TOKEN_GITLAB@git.ais.uni-bonn.de/athome/nimbro_utils.git" /root/colcon_ws/src/nimbro_utils
COPY . /root/colcon_ws/src/nimbro_camera_ids

WORKDIR /root/colcon_ws
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install
WORKDIR /root/colcon_ws/src/nimbro_camera_ids

ENTRYPOINT ["/bin/bash", "-c"]
CMD ["scripts/start_all.sh"]
