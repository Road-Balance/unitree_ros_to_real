```
cd /dev/input
ls | grep js0
```

조이스틱 인식이 잘 안될 때는 모드를 바꿔본다.

# mode

* X => stand mode (Btn[2], a1_mode 1)
```
left <>  = roll
left ^⌵ = pitch
right <> = yaw
```

* A => trot (Btn[0], a1_mode 2)

mode = 2
gaitType = 1

```
left <>  = velocity[1]
left ^⌵ = velocity[0]
right <> = yawSpeed ? velocity[2]?

Btn ^⌵ = bodyHeight Up Down (Default 0.1)
Btn LB RB = footRaiseHeight Up Down (Default 0.1)
```

* B => Trot Runnung (Btn[1], a1_mode 3)
mode = 2
gaitType = 2
```
left <>  = velocity[1]
left ^⌵ = velocity[0]
right <> = yawSpeed ? velocity[2]?

Btn ^⌵ = bodyHeight Up Down (Default 0.1)
Btn LB RB = footRaiseHeight Up Down (Default 0.1)
```

* Y => Stairs Climbing (Btn[3], a1_mode 4)
mode = 2 
gaitType = 3
```
left <>  = velocity[1]
left ^⌵ = velocity[0]
right <> = yawSpeed ? velocity[2]?

Btn ^⌵ = bodyHeight Up Down (Default 0.1)
Btn LB RB = footRaiseHeight Up Down (Default 0.1)
```