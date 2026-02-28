{
  description = "The Green Machine's ZTLDR";
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };
  outputs = {
    self,
    nixpkgs,
    flake-utils,
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = import nixpkgs {
        inherit system;
        config.allowUnfree = true;
      };

      libPath = pkgs.lib.makeLibraryPath (with pkgs; [
        glfw
        libGL
        libGLU
        libglvnd
        mesa
        xorg.libX11
        xorg.libXrandr
        xorg.libXcursor
        xorg.libXi
        xorg.libXinerama
        xorg.libXxf86vm
        xorg.libXext
        xorg.libXrender
        xorg.libXfixes
        wayland
        libxkbcommon
        vulkan-loader
        stdenv.cc.cc.lib
        zlib
      ]);

      mkGradleApp = task:
        pkgs.writeShellApplication {
          name = "gradle-${task}";
          runtimeInputs = with pkgs; [
            jdk17
            gradle
          ];
          text = ''
            set -euo pipefail
            export JAVA_HOME="${pkgs.jdk17}"

            if [ -d /run/opengl-driver/lib ]; then
              export LD_LIBRARY_PATH="/run/opengl-driver/lib:${libPath}:''${LD_LIBRARY_PATH-}"
            else
              export LD_LIBRARY_PATH="${libPath}:''${LD_LIBRARY_PATH-}"
            fi
            if [ -d /run/opengl-driver/lib/dri ]; then
              export LIBGL_DRIVERS_PATH="/run/opengl-driver/lib/dri"
            else
              export LIBGL_DRIVERS_PATH="${pkgs.mesa}/lib/dri"
            fi
            if [ -d /run/opengl-driver/share/glvnd/egl_vendor.d ]; then
              export __EGL_VENDOR_LIBRARY_DIRS="/run/opengl-driver/share/glvnd/egl_vendor.d"
            fi
            if [ -d /run/opengl-driver/share/glvnd/glx_vendor.d ]; then
              export __GLX_VENDOR_LIBRARY_DIRS="/run/opengl-driver/share/glvnd/glx_vendor.d"
            fi

            if [ -x "./gradlew" ]; then
              exec ./gradlew --no-daemon ${task} "$@"
            else
              exec gradle --no-daemon ${task} "$@"
            fi
          '';
        };

      ideaPkg = pkgs.jetbrains.idea;

      ideaApp = pkgs.writeShellApplication {
        name = "idea";
        runtimeInputs = [ideaPkg];
        text = ''
          set -euo pipefail
          export JAVA_HOME="${pkgs.jdk17}"
          if [ -d /run/opengl-driver/lib ]; then
            export LD_LIBRARY_PATH="/run/opengl-driver/lib:${libPath}:''${LD_LIBRARY_PATH-}"
          else
            export LD_LIBRARY_PATH="${libPath}:''${LD_LIBRARY_PATH-}"
          fi
          if [ -d /run/opengl-driver/lib/dri ]; then
            export LIBGL_DRIVERS_PATH="/run/opengl-driver/lib/dri"
          else
            export LIBGL_DRIVERS_PATH="${pkgs.mesa}/lib/dri"
          fi
          exec "${ideaPkg}/bin/idea" "$@"
        '';
      };
    in {
      devShells.default = pkgs.mkShell {
        packages = with pkgs; [
          jdk17
          gradle

          glfw
          libGL
          libGLU
          libglvnd
          mesa
          xorg.libX11
          xorg.libXrandr
          xorg.libXcursor
          xorg.libXi
          xorg.libXinerama
          xorg.libXxf86vm
          xorg.libXext
          xorg.libXrender
          xorg.libXfixes
          wayland
          libxkbcommon
          vulkan-loader

          stdenv.cc.cc.lib
          zlib
        ];

        shellHook = ''
          export JAVA_HOME="${pkgs.jdk17}"

          # Keep host OpenGL drivers visible inside the Nix shell.
          if [ -d /run/opengl-driver/lib ]; then
            export LD_LIBRARY_PATH="/run/opengl-driver/lib:${libPath}:''${LD_LIBRARY_PATH-}"
          else
            export LD_LIBRARY_PATH="${libPath}:''${LD_LIBRARY_PATH-}"
          fi
          if [ -d /run/opengl-driver/lib/dri ]; then
            export LIBGL_DRIVERS_PATH="/run/opengl-driver/lib/dri"
          else
            export LIBGL_DRIVERS_PATH="${pkgs.mesa}/lib/dri"
          fi
          if [ -d /run/opengl-driver/share/glvnd/egl_vendor.d ]; then
            export __EGL_VENDOR_LIBRARY_DIRS="/run/opengl-driver/share/glvnd/egl_vendor.d"
          fi
          if [ -d /run/opengl-driver/share/glvnd/glx_vendor.d ]; then
            export __GLX_VENDOR_LIBRARY_DIRS="/run/opengl-driver/share/glvnd/glx_vendor.d"
          fi
        '';
      };
      packages = {
        runZTLDR = mkGradleApp "runZTLDR";
        idea = ideaApp;
      };
      apps = {
        runZTLDR = flake-utils.lib.mkApp {drv = self.packages.${system}.runZTLDR;};
        idea = flake-utils.lib.mkApp {drv = self.packages.${system}.idea;};
      };
    });
}
