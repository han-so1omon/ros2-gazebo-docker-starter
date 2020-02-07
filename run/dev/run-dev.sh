docker run -it \
  --user $(id -u $USER) \
  --workdir="/home/$USER" \
  --volume="/l/p/lenta/refrain/:/home/$USER" \
  --volume="/etc/group:/etc/group:ro" \
  --volume="/etc/passwd:/etc/passwd:ro" \
  --volume="/etc/shadow:/etc/shadow:ro" \
  --volume="/etc/sudoers.d:/etc/sudoers.d:ro" \
  errcsool/refrain-dev \
  /bin/bash
