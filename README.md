#Falcon  

##Objective

The project is aimed at providing a framework for which controllers can be developed for the Novint Falcon with ease. It is still undetermined at the moment the architectural setup of the project, however the focus will be on making it easy to interchange between controllers with the greatest performance.

##Prerequisite

+ [libnifalcon 1.0.1](http://qdot.github.io/libnifalcon/downloads.html)
+ [boost c++ 1.49.0](http://sourceforge.net/projects/boost/files/boost/1.49.0/)

##Downloading

###Linux/Mac OS X

``` sh
$ git clone git://github.com/RReichert/Falcon.git
$ cd falcon
```

###Windows

```
IN PROGRESS
```

##Compiling

###Linux/Mac OS X

``` sh
$ mkdir build
$ cd build
$ cmake -DCMAKE_BUILD_TYPE=Release ..
$ make
```

###Windows

```
IN PROGRESSS
```

##Executing

###Linux/Mac OS X

``` sh
$ cd bin
$ ./falcon
```

###Windows

```
IN PROGRESS
```

##Known Issues

+ [bug#7241](https://svn.boost.org/trac/boost/ticket/7241): forced me to link with Boost System library, even through we don't use it
+ unable to dynamically linking with boost library, because of a bug with the new Log library.

##Report Bugs/Support

If there are any questions, noticed a bug, or would like to criticise my code, please feel free to email (&lt;mylastname &gt;.&lt;myfirstname&gt;@gmail.com - done for spamming reasons) and I will get back to you as soon as possible.
