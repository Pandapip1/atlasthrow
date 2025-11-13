{
  inputs = {
    ros-overlay = {
      url = "github:lopsided98/nix-ros-overlay";
      inputs.flake-utils.follows = "flake-utils";
    };
    nixpkgs.url = "github:lopsided98/nixpkgs/nix-ros"; # TODO: Don't use a nixpkgs fork
    systems = {
      url = "path:./systems/default.nix";
      flake = false;
    };
    flake-parts = {
      url = "github:hercules-ci/flake-parts";
      inputs.nixpkgs-lib.follows = "nixpkgs";
    };
    treefmt-nix = {
      url = "github:numtide/treefmt-nix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # Dependencies
    flake-utils = {
      url = "github:numtide/flake-utils";
      inputs.systems.follows = "systems";
    };
  };

  outputs =
    inputs:
    let
      inherit (inputs) self;
      inherit (inputs.nixpkgs) lib;
      distro = "jazzy";
    in
    inputs.flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [
        inputs.treefmt-nix.flakeModule
      ];
      flake = {
        overlays.default = final: prev: {
          rosPackages = lib.mapAttrs (
            distro': _:
            prev.rosPackages.${distro'}
            // (lib.mapAttrs'
              (name: value: {
                name = lib.replaceStrings [ "_" ] [ "-" ] name;
                inherit value;
              })
              (
                lib.packagesFromDirectoryRecursive {
                  callPackage = lib.callPackageWith (final // final.rosPackages.${distro'});
                  directory = ./. + "/src";
                }
              )
            )
          ) prev.rosPackages;
        };
      };
      perSystem =
        {
          self',
          config,
          pkgs,
          system,
          ...
        }:
        {
          packages = {
            rosEnv = pkgs.rosPackages.${distro}.buildEnv {
              paths =
                with pkgs.rosPackages.${distro};
                [
                  # ROS Core Dependencies
                  ros-core
                  ament-cmake
                  ament-cmake-core
                  python-cmake-module
                  # Atlas deps
                  xacro
                  robot-state-publisher
                  joint-state-publisher-gui
                  rviz2
                  # Plotjoints.py deps
                  rosbag2
                  rosbag2-py
                  # More deps
                  urdfdom-py
                ]
                ++ (with pkgs; [
                  colcon
                ]);
            };
          };
          treefmt = {
            programs = {
              autocorrect.enable = true;
              nixfmt.enable = true;
              clang-format.enable = true;
            };
            settings.formatter.markdownlint = {
              enable = true;
              command = lib.getExe pkgs.markdownlint-cli2;
              options = [ "--fix" ];
              includes = [ "*.md" ];
            };
          };
          devShells.default = pkgs.mkShell {
            nativeBuildInputs =
              [ self'.packages.rosEnv ]
              ++ (with pkgs; [
                httplib
              ]);
            buildInputs = (
              with pkgs;
              [
                curl
                # Add Qt dependencies
                qt5.qtbase
                qt5.qtwayland
                qt5.qtx11extras
                xorg.libX11
                xorg.libxcb
                xorg.libXcursor
                xorg.libXext
                xorg.libXi
                xorg.libXrender
                xorg.libXtst
              ]
            ) ++ (
              with pkgs.python3Packages; [
                python
                numpy
                scipy
                matplotlib
              ]
            );
            shellHook = ''
              # For matplotlib:
              # export QT_QPA_PLATFORM=wayland-egl
              export QT_QPA_PLATFORM=xcb
              colcon build --symlink-install
              source install/setup.bash
            '';
          };
          # https://flake.parts/overlays.html
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays = [
              inputs.ros-overlay.overlays.default
              self.overlays.default
            ];
            config = { };
          };
        };
      systems = import inputs.systems.outPath;
    };
  nixConfig = {
    extra-substituters = [ "https://ros.cachix.org" ];
    extra-trusted-public-keys = [ "ros.cachix.org-1:dSyZxI8geDCJrwgvCOHDoAfOm5sV1wCPjBkKL+38Rvo=" ];
  };
}
