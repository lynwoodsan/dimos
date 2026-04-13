{
  description = "DimOS Arduino support — bridge binary + Arduino toolchain";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};

        # LCM with single output (avoids split-output path issues)
        lcmFull = pkgs.lcm.overrideAttrs (old: {
          outputs = [ "out" ];
          postInstall = "";
        });

        # The generic serial↔LCM bridge
        arduino_bridge = pkgs.stdenv.mkDerivation {
          pname = "arduino_bridge";
          version = "0.1.0";
          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ lcmFull pkgs.glib ];

          cmakeFlags = [
            "-DDIMOS_LCM_DIR=${dimos-lcm}"
          ];

          # CMakeLists.txt is in cpp/ subdirectory
          cmakeDir = "../cpp";

          installPhase = ''
            mkdir -p $out/bin
            cp arduino_bridge $out/bin/
          '';
        };

      in {
        packages = {
          inherit arduino_bridge;
          default = arduino_bridge;
        };

        devShells.default = pkgs.mkShell {
          packages = [
            arduino_bridge
            pkgs.arduino-cli
            pkgs.avrdude
            pkgs.picocom
          ];
        };
      });
}
