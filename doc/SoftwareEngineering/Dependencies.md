
Dependencies expressed at the cmake level can be retrieved, on Unix likes, with the following command:
  `find . -name CMakeLists.txt -exec grep -i FIND_PACKAGE {} \; | tr -d ' ' | tr -d ')' | tr -d '\011'| grep -v "#" |sed "s/FIND_PACKAGE(//" | sed "s/REQUIRED//" | sed "s/find_package(//"| sort -u`
