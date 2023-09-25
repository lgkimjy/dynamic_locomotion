# Clone External Libraries here, including MuJoCo

```console
// tag:= 2.3.2 ~ 2.3.7 (choose MuJoCo release version on own taste)
// under 2.3.2 is not working due to the missing header, "platform_ui_adapter.h"
$ git clone -b [tag name] https://github.com/deepmind/mujoco.git

// If you want to use modified MuJoCo, then just fork the original MuJoCo,
// , modify there and add it to the submodule
$ git submodule add [url: own git repo]
```

