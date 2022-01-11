# Biomechatronics Lab C++ Style Guide 
Based on https://github.com/isocpp/CppCoreGuidelines
and https://www.python.org/dev/peps/pep-0008/


## General 
- Avoid extra whitespace
- 4 space tab
- DON'T place two statements on the same line
- DON'T use void as an argument type
- If globals exist still pass them by reference


## Commenting
- Use docstring (/*  */) for namespaces, types, methods, functions, and constants use explination before definition.
- State intent in comments, not just a restatement of what is readible in the code.
- Comment code as you go so you know what you were trying to do.


## Naming
- [ ] TODO Chance: What are your thoughts on this one, if they are all passed I think this is less of an issue, however a lot of the board.h files define constants like the macro?  The macro style for constants is standard in Python, but is counter to C++ style, but is still often used.  Alternativly we could use a namespace so board::can_rx.
- ALL_CAPS for macro names only, this avoid thinking something is a macro when it is subject to type rules.
- Globals should use a leading "g_", hopefully you don't have many of these
- [ ] TODO let's chat: enum class vs typedef?

### Functions
- Function names should be lowercase, with words separated by underscores as necessary to improve readability.

### Classes
- Class names should normally use the CapWords convention.
- Methods should use function naming convention.
- Private members should use a leading underscores 

### Types
- Type variable names should us CapWord Convention

### Variables
- Never use the characters 'l' (lowercase letter el), 'O' (uppercase letter oh), or 'I' (uppercase letter eye) as single character variable names.
- Variable names should follow function naming convention
- DON'T include type in the name, e.g. int_use_this






