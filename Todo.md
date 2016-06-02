 * Reduce number of warnings:
   * [DGTal policy](https://github.com/DGtal-team/DGtal/pull/1182) require the build of a PR to be free of warnings (refer e.g. to the [checklist of this PR](http://stackoverflow.com/questions/15500143/clang-promoting-all-warnings-to-errors-except-for)
   * DGTal achieves by setting the compiler in "warnings as debug" mode when building the PR (and letting the CI trap the error as a classic build fail)
   * Both clang and GCC seem to accept such a compile option (`--Werror`):
     * [clang -Werror](http://stackoverflow.com/questions/15500143/clang-promoting-all-warnings-to-errors-except-for) although not easily documented seems to work
     * Start looking [here whether gcc does the same](http://stackoverflow.com/questions/8466295/gcc-and-clang-warnings-errors-flags)
   * Still for [clang the `-Wno-error=foo`](http://stackoverflow.com/questions/15500143/clang-promoting-all-warnings-to-errors-except-for) turns warning “foo” even if -Werror is specified.   
