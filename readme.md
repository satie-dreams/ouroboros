# Guitar Looper 

The main concept of the project is to create a professional guitar looper that can repeating music seamlessly and endlessly.

### Team:

* Maletskyi Denys (@maletsden)
* Maxim Zhuravinsky(@ogoremeni)
* Markiyan Novosad (@mnovosad1095)


### Characteristics

##### Audio

- 12-bit audio in/out
- 44100 samples/sec

##### Control

- 4 buttons to select channels
- 1 switcher for a sample duration

##### Connection

- INPUT (MONO) jack: 1/4-inch phone type
- OUTPUT (MONO) jack: 1/4-inch phone type

##### Output

- Micro SD card that saves all recorded data

##### Miscrocontoller

-  PSoC 6 (CY8CPROTO-062-4343W board)



### Instalation

Get all necessary dependencies:
```{sh}
make getlibs
```

Build project:
```{sh}
make program TARGET=CY8CPROTO-062-4343W TOOLCHAIN=GCC_ARM
```
