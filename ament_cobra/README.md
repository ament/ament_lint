# Issues

* Output from the various rule checkers is irregular, making it near impossible to parse into the xunit format
* Adding include directories causes Cobra to invoke the preprocessor, however, we would need (ament-level) info for -D and -U options
    * Fails attempting to process Windows headers, for example, when they should be skipped
* Invoking the MISRA checks invokes the preprocessor
* There are many false positives because of the token-based rules
* Rules don't comprehend more recent C++ syntax
* Cobra doesn't support excluding specific files or directories (like a test subdirectory, for example)

