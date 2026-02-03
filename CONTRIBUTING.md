# Contributing to VI-SLAM

Thank you for your interest in contributing to VI-SLAM! This document provides guidelines and instructions for contributing to the project.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [How to Contribute](#how-to-contribute)
- [Development Setup](#development-setup)
- [Coding Standards](#coding-standards)
- [Commit Message Guidelines](#commit-message-guidelines)
- [Pull Request Process](#pull-request-process)
- [Testing Requirements](#testing-requirements)
- [Issue Reporting](#issue-reporting)

## Code of Conduct

This project adheres to the Contributor Covenant Code of Conduct. By participating, you are expected to uphold this code. Please be respectful and considerate in all interactions.

## Getting Started

1. **Fork the repository** on GitHub
2. **Clone your fork** locally:
   ```bash
   git clone https://github.com/YOUR_USERNAME/vi_slam.git
   cd vi_slam
   ```
3. **Add the upstream remote**:
   ```bash
   git remote add upstream https://github.com/kcenon/vi_slam.git
   ```
4. **Create a feature branch** from `main`:
   ```bash
   git checkout -b feat/your-feature-name
   ```

## How to Contribute

### Reporting Bugs

- Search existing issues to avoid duplicates
- Use the bug report template when creating a new issue
- Include steps to reproduce, expected behavior, and actual behavior
- Provide system information (OS, compiler version, etc.)

### Suggesting Features

- Open an issue with the feature request template
- Describe the use case and benefits
- Consider implementation complexity

### Code Contributions

1. Find an open issue or create one describing your contribution
2. Comment on the issue to indicate you're working on it
3. Follow the development workflow described below

## Development Setup

### Prerequisites

**For PC Client (C++):**
- CMake 3.16+
- C++17 compatible compiler (GCC 9+, Clang 10+, MSVC 2019+)
- OpenCV 4.5+
- Eigen3
- Python 3.8+

**For Android:**
- Android Studio Arctic Fox or later
- JDK 17
- Android SDK 34

### Building the Project

**PC Client:**
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

**Android App:**
```bash
cd android
./gradlew build
```

See the [README.md](README.md) for detailed setup instructions.

## Coding Standards

### C++ Code Style

- Use modern C++17 features appropriately
- Follow the existing code style in the repository
- Use `.clang-format` for automatic formatting:
  ```bash
  clang-format -i src/**/*.cpp include/**/*.hpp
  ```
- Naming conventions:
  - Classes: `PascalCase` (e.g., `SLAMEngine`)
  - Functions: `camelCase` (e.g., `processFrame`)
  - Variables: `camelCase` (e.g., `frameCount`)
  - Constants: `UPPER_SNAKE_CASE` (e.g., `MAX_FEATURES`)
  - Namespaces: `snake_case` (e.g., `vi_slam`)

### Kotlin/Android Code Style

- Follow [Kotlin Coding Conventions](https://kotlinlang.org/docs/coding-conventions.html)
- Use Android Studio's built-in formatter
- Follow MVVM architecture patterns

### Documentation

- Document all public APIs with Doxygen-style comments (C++)
- Document all public classes and functions with KDoc (Kotlin)
- Keep comments concise and meaningful

## Commit Message Guidelines

We follow the [Conventional Commits](https://www.conventionalcommits.org/) specification.

### Format

```
type(scope): description

[optional body]

[optional footer]
```

### Types

| Type | Description |
|------|-------------|
| `feat` | New feature |
| `fix` | Bug fix |
| `docs` | Documentation changes |
| `style` | Formatting, missing semicolons, etc. |
| `refactor` | Code change that neither fixes a bug nor adds a feature |
| `perf` | Performance improvement |
| `test` | Adding or modifying tests |
| `build` | Build system or dependency changes |
| `ci` | CI configuration changes |
| `chore` | Other changes that don't modify src or test files |

### Examples

```bash
# Good examples
feat(slam): add loop closure detection
fix(android): resolve camera permission crash
docs(readme): update installation instructions
refactor(pc): simplify frame processing pipeline
test(slam): add unit tests for feature extraction

# Bad examples (avoid these)
fixed bug                    # No type, vague description
FEAT: Add feature           # Wrong capitalization
feat(slam): Add loop closure # Description should be lowercase
```

### Scope

Use the component or module name:
- `slam` - SLAM processing
- `android` - Android application
- `pc` - PC client
- `visualizer` - Visualization components
- `test` - Test infrastructure
- `ci` - CI/CD configuration
- `docs` - Documentation

## Pull Request Process

### Before Submitting

1. **Sync with upstream**:
   ```bash
   git fetch upstream
   git rebase upstream/main
   ```

2. **Run tests locally**:
   ```bash
   # PC Client
   cd build && ctest --output-on-failure

   # Android
   cd android && ./gradlew test
   ```

3. **Ensure code passes linting**:
   ```bash
   # C++ formatting check
   clang-format --dry-run --Werror src/**/*.cpp include/**/*.hpp
   ```

### PR Guidelines

- Fill out the PR template completely
- Link to the related issue using `Closes #123`
- Keep PRs focused on a single change
- Request review from appropriate team members
- Address all review comments

### PR Size Guidelines

| Size | Lines Changed | Recommendation |
|------|---------------|----------------|
| XS | < 50 | Ideal for quick reviews |
| S | 50-200 | Preferred size |
| M | 200-500 | Consider splitting |
| L | 500+ | Must split into smaller PRs |

### After Submission

- CI must pass before merge
- At least one approval required
- Maintainers may request changes
- Squash merge is preferred for clean history

## Testing Requirements

### PC Client (C++)

- All new code must have unit tests
- Maintain test coverage above 80%
- Use Google Test framework:
  ```cpp
  TEST(FeatureExtractor, ExtractsCorrectNumberOfFeatures) {
      FeatureExtractor extractor;
      auto features = extractor.extract(testImage);
      EXPECT_GE(features.size(), 100);
  }
  ```

### Android (Kotlin)

- Write unit tests for ViewModels and Repositories
- Use MockK for mocking dependencies:
  ```kotlin
  @Test
  fun `processFrame emits correct state`() {
      val viewModel = MainViewModel(mockRepository)
      // Test implementation
  }
  ```

### Running Tests

```bash
# PC Client - All tests
cd build && ctest --output-on-failure

# PC Client - Specific test
./build/tests/test_slam_engine

# Android - Unit tests
cd android && ./gradlew test

# Android - Instrumentation tests
cd android && ./gradlew connectedAndroidTest
```

## Issue Reporting

### Bug Reports

Include:
- Clear, descriptive title
- Steps to reproduce
- Expected vs actual behavior
- System information
- Relevant logs or screenshots

### Feature Requests

Include:
- Clear description of the feature
- Use case and motivation
- Potential implementation approach (optional)

## Questions?

If you have questions about contributing, feel free to:
- Open a discussion on GitHub
- Create an issue with the `question` label

Thank you for contributing to VI-SLAM!
