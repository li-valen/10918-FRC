# Julia R. Masterman High School FRC Robotics

This repository contains the robot code used by the Julia R. Masterman High School FRC Robotics team. All team members will collaborate here during the season.

## Contribution Workflow

- **Personal branches only**: Each person should do their work on their own branch.
  - Suggested naming: `yourname/feature-short-name` or `yourname/fix-short-name`.
- **Pull requests to `main`**: When you want your code merged into the main working robot, open a PR targeting `main`.
  - **Tag @li-valen** in your PR description or as a reviewer so it’s seen quickly.
  - Keep PRs focused and reasonably small for faster review.
- **Reviews & merges**:
  - At least one reviewer approval is required before merge (coach/lead may require more during competition season).
  - Resolve all review comments and make sure checks pass before merge.
- **Commit messages**: Write clear, concise messages (e.g., "Drive: smooth ramp on motor output").

## Repo Layout

```
10918-FRC/
  build.gradle
  gradlew, gradlew.bat
  settings.gradle
  src/
    main/
      java/
        frc/robot/
          Main.java
          Robot.java
      deploy/
        example.txt
  vendordeps/
  WPILib-License.md
```

- `src/main/java/frc/robot/Robot.java`: Main robot class where robot lifecycle methods live.
- `src/main/java/frc/robot/Main.java`: Program entry point used by WPILib.
- `src/main/deploy/`: Files deployed to the roboRIO (e.g., trajectories, configs).
- `vendordeps/`: Vendor dependency JSONs (CTRE, REV, etc.).

## Prerequisites

- WPILib (VS Code extension or your preferred IDE setup) installed for the current season.
- Java 17+ (installed automatically by WPILib tools if using the WPILib VS Code bundle).
- Git access to this repository.

## First-Time Setup

1. Clone the repo:
   ```bash
   git clone https://github.com/your-org-or-user/10918-FRC.git
   cd 10918-FRC/10918-FRC
   ```
2. Open the folder in VS Code with the WPILib extension or your preferred IDE.
3. Let the WPILib extension detect the project and set up the toolchain.

## Typical Developer Flow

1. Create your branch:
   ```bash
   git checkout -b yourname/feature-short-name
   ```
2. Make changes, build, and test locally.
3. Commit and push:
   ```bash
   git add -A
   git commit -m "Subsystem: brief change description"
   git push -u origin yourname/feature-short-name
   ```
4. Open a Pull Request into `main` and **tag @li-valen** for review.

## Build, Simulate, and Deploy

All commands should be run from the project directory containing `gradlew`.

- Build (compile and run checks):
  ```bash
  ./gradlew build
  ```
- Run desktop simulation (if applicable to current code):
  ```bash
  ./gradlew simulateJava
  ```
- Deploy to roboRIO (robot must be on the same network):
  ```bash
  ./gradlew deploy
  ```
- Clean builds:
  ```bash
  ./gradlew clean
  ```

If you use the WPILib VS Code extension, these actions are also available via the Command Palette.

## Coding Guidelines

- Prefer readability and clear naming over cleverness.
- Keep changes small and focused; avoid mixing unrelated edits in one PR.
- Add comments only where the intent is not obvious from code.
- Keep robot periodic loops efficient; avoid blocking calls in `robotPeriodic`/`teleopPeriodic`.
- If introducing new subsystems/commands, follow standard WPILib structure and organize code under `frc/robot` by feature.

## Vendor Dependencies

- Place vendor JSON files in `vendordeps/` and re-run a build. Commit the JSONs.
- Keep vendor libraries up to date and verify compatibility with the season’s WPILib.

## Troubleshooting

- If Gradle fails, try `./gradlew --version` and `./gradlew clean build`.
- Ensure the roboRIO is imaged for the current season and that your team number is set in WPILib preferences.
- Check firewall/network if deploy cannot find the roboRIO.

## License

This project builds on WPILib. See `WPILib-License.md` for the WPILib license.

## Support and Questions

- Open a GitHub Issue for bugs or feature requests.
- For PRs that need attention, remember to **tag @li-valen**.
