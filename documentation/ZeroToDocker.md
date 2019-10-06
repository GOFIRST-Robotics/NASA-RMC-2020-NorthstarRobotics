Setup for Building
===
1. Install Docker:

```
$ sudo apt-get update
$ sudo apt-get install docker-ce docker-ce-cli containerd.io
```

2. Add yourself to the `docker` group:
```
$ usermod -a -G docker <your user name>
```
Then log out and back in.

3. Run `./rmcbuild` in the main directory

That's it! You have now built the project using an environment exactly the same as the one that is running on the Jetson.
If you want to interact with the built images, you can use the `rmcrun` script to start a bash terminal or do anything else.