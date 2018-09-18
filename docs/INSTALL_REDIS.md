# Installing Redis (intended mostly for Windows 10 users)
If you're using a non-Unix operating system (i.e. Windows 10) as your main platform for development, **don't if you can help it**. It's much easier to use Unix-based OSes like macOS or any Linux distribution for development, and most installation guides will be written with Unix systems in mind. That being said, if you need to use Windows because of software compatibility issues or because you can't dual boot on your computer for whatever reason, you should be using Windows 10. This installation guide walks you through installing Redis via the Windows Subsystem for Linux, which only works on Windows 10. If you're using another version, [you're on your own](https://www.youtube.com/watch?v=IgAsNtmlQJU).

## For Non-Windows Users
Thank you for making the right choice. As a reward, your installation will only take 4 steps and won't involve a mysterious `make: *** [setup] Error 2` when you try to build Redis. Follow the [Redis Quick Start](https://redis.io/topics/quickstart) to get up and running.

## Install Windows Subsystem for Linux
1. Open PowerShell as an Administrator (search for PowerShell and right click->Run as Administrator) and run the following: `Enable-WindowsOptionalFeature -Online -FeatureName Microsoft-Windows-Subsystem-Linux`.
2. Restart your computer.
3. Go to the Windows Store and search for "Ubuntu," the page that comes up should look like the following:
![](https://docs.microsoft.com/en-us/windows/wsl/media/ubuntustore.png)
4. Let that distro install, it should only take a few minutes.
5. Once the distro finishes installing you should be prompted to create a new Unix username and password; pick something that you'll remember for when you need to install things or otherwise elevate permissions. If you aren't prompted to do so, just open Ubuntu Bash and it should prompt you.
6. One Ubuntu Bash is set up, open it and run `sudo apt update && sudo apt upgrade`. This might take a while.
7. Once you've finished upgrading your packages, run `sudo apt-get install gcc`. You're done with this portion of the installation. You get one party popper (+1 Party Popper 🎉).

## Install Redis
1. Run `wget http://download.redis.io/redis-stable.tar.gz` to download the Redis source. You should probably do this outside of the root directory - where Ubuntu Bash starts you off - to avoid using `sudo`, but it's fine if you use `sudo wget http://download.redis.io/redis-stable.tar.gz` here.
2. Once Redis is done downloading, run `tar xvzf redis-stable.tar.gz`. If you downloaded Redis to the root directory, make sure to prefix this with `sudo` too (`sudo tar xvzf redis-stable.tar.gz`), then `cd redis-stable`.
3. You can now run `make`, which will build Redis from source. If `make` throws an error, try using `make MALLOC=libc`.
4. Once everything is built, you can run `sudo make install` to install the built binaries.
5. After installation is complete, you'll want to copy redis.conf over to /usr/local/bin using `sudo cp redis.conf /usr/local/bin`.
6. Finally, `cd utils` and `sudo ./install_server.sh`. Click Enter a bunch of times to keep the default values. Once this is done, try running `redis-server`. If you see something like this, you're ready to get started:
```
                _._
           _.-``__ ''-._
      _.-``    `.  `_.  ''-._           Redis 4.0.11 (00000000/0) 64 bit
  .-`` .-```.  ```\/    _.,_ ''-._
 (    '      ,       .-`  | `,    )     Running in standalone mode
 |`-._`-...-` __...-.``-._|'` _.-'|     Port: 6379
 |    `-._   `._    /     _.-'    |     PID: 28715
  `-._    `-._  `-./  _.-'    _.-'
 |`-._`-._    `-.__.-'    _.-'_.-'|
 |    `-._`-._        _.-'_.-'    |           http://redis.io
  `-._    `-._`-.__.-'_.-'    _.-'
 |`-._`-._    `-.__.-'    _.-'_.-'|
 |    `-._`-._        _.-'_.-'    |
  `-._    `-._`-.__.-'_.-'    _.-'
      `-._    `-.__.-'    _.-'
          `-._        _.-'
              `-.__.-'
```
**Congratulations!** You've installed `redis-server` and `redis-cli`. Take two party poppers this time (+2 Party Popper 🎉).

Now read the rest of the setup instructions in the README and get to work.
