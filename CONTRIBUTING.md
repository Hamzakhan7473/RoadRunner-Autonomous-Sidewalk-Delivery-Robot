# Contributing to India-Ready Delivery Robot

Thank you for your interest in contributing to the India-Ready Delivery Robot project! This document provides guidelines for contributing to our open-source initiative.

## 🤝 How to Contribute

### Reporting Issues
- Use GitHub Issues to report bugs or request features
- Provide detailed descriptions and reproduction steps
- Include system information and logs when relevant
- Use appropriate labels for categorization

### Code Contributions
- Fork the repository
- Create a feature branch from `main`
- Make your changes with clear commit messages
- Add tests for new functionality
- Ensure code follows our style guidelines
- Submit a pull request with detailed description

### Documentation
- Improve existing documentation
- Add examples and tutorials
- Translate documentation to Indian languages
- Create video tutorials and demos

## 🛠️ Development Setup

### Prerequisites
- ROS Noetic (Ubuntu 20.04)
- Python 3.8+
- Flutter SDK
- Git

### Installation
```bash
# Clone the repository
git clone https://github.com/Hamzakhan7473/RoadRunner-Autonomous-Sidewalk-Delivery-Robot.git
cd RoadRunner-Autonomous-Sidewalk-Delivery-Robot

# Setup ROS workspace
cd ros_ws
catkin_make
source devel/setup.bash

# Setup Flutter app
cd ../flutter_app
flutter pub get
```

## 📋 Coding Standards

### Python
- Follow PEP 8 style guidelines
- Use type hints where appropriate
- Add docstrings for functions and classes
- Write unit tests for new code

### ROS
- Follow ROS naming conventions
- Use proper message types
- Add launch file documentation
- Include parameter validation

### Flutter
- Follow Dart style guidelines
- Use proper widget composition
- Add localization support
- Write widget tests

## 🧪 Testing

### Unit Tests
- Write tests for all new functionality
- Maintain >80% code coverage
- Use appropriate testing frameworks
- Test edge cases and error conditions

### Integration Tests
- Test ROS node interactions
- Validate Flutter app functionality
- Test hardware-software integration
- Perform end-to-end testing

## 📝 Commit Guidelines

### Commit Message Format
```
type(scope): description

[optional body]

[optional footer]
```

### Types
- `feat`: New feature
- `fix`: Bug fix
- `docs`: Documentation changes
- `style`: Code style changes
- `refactor`: Code refactoring
- `test`: Test additions/changes
- `chore`: Build/tooling changes

### Examples
```
feat(navigation): add social navigation behaviors
fix(perception): resolve obstacle detection issues
docs(readme): update installation instructions
```

## 🔍 Review Process

### Pull Request Requirements
- Clear description of changes
- Link to related issues
- Passing tests and CI checks
- Updated documentation
- Code review approval

### Review Criteria
- Code quality and style
- Test coverage
- Documentation completeness
- Performance implications
- Security considerations

## 🌍 Localization

### Adding New Languages
- Add language files to `flutter_app/lib/localization/`
- Update `AppLocalizationsDelegate`
- Test UI with new language
- Update documentation

### Translation Guidelines
- Use culturally appropriate terms
- Consider regional variations
- Maintain consistency across UI
- Test with native speakers

## 🚀 Release Process

### Version Numbering
- Follow Semantic Versioning (MAJOR.MINOR.PATCH)
- Update version in all relevant files
- Create release notes
- Tag releases appropriately

### Release Checklist
- [ ] All tests passing
- [ ] Documentation updated
- [ ] Version numbers updated
- [ ] Release notes prepared
- [ ] Security review completed

## 📞 Support

### Getting Help
- Check existing documentation
- Search GitHub Issues
- Join our community discussions
- Contact maintainers directly

### Community Guidelines
- Be respectful and inclusive
- Help others learn and grow
- Share knowledge and experiences
- Follow our code of conduct

## 🎯 Areas for Contribution

### High Priority
- Indian traffic dataset creation
- Social navigation algorithms
- Hardware optimization
- Cost reduction strategies
- Safety improvements

### Medium Priority
- Additional language support
- Performance optimizations
- Documentation improvements
- Testing framework enhancements
- Deployment automation

### Low Priority
- UI/UX improvements
- Additional sensor support
- Advanced AI features
- International expansion
- Research collaborations

## 📄 License

By contributing to this project, you agree that your contributions will be licensed under the MIT License.

## 🙏 Recognition

Contributors will be recognized in:
- CONTRIBUTORS.md file
- Release notes
- Project documentation
- Community acknowledgments

Thank you for contributing to the India-Ready Delivery Robot project! Together, we can build autonomous delivery solutions that work for Indian urban environments.
