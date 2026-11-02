# Third-party dependencies

This repository does not bundle the following dependencies:

- MuJoCo — Apache License 2.0
- Eclipse Zenoh / zenoh-python — used by the Platform communication tools;
  see the license shipped with the selected Zenoh distribution
- Protocol Buffers — BSD 3-Clause License; used by the Platform Python messages

Consumers are responsible for obtaining these dependencies and complying with
their licenses. Robot MJCF files, meshes, textures, and other model packages are
not part of this SDK unless explicitly added with compatible licensing.

This repository bundles msgpack11 (Copyright 2016 Masahiro Wada) under the MIT
License. Its license text is available at `third_party/msgpack11/LICENSE.txt`.
