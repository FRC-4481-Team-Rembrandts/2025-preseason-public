package frc.lib.hardware.ctre;

public class PigeonHelper {

    /*
    Hier woont een duif
                     ░░░░
              ░░░░░░░░▒▒░░░░░░
          ░░░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
        ░░░░▒▒▒▒▒▒▒▒▒▒░░░░▒▒▒▒▒▒░░
      ░░░░▒▒▒▒▒▒▒▒▒▒░░▒▒▒▒░░▒▒▒▒▒▒
      ░░░░▒▒▒▒▒▒▒▒▒▒░░  ▓▓▒▒░░▒▒▒▒▒▒
      ▒▒░░░░▒▒▒▒▒▒▒▒░░▓▓▓▓▓▓░░▒▒▒▒▒▒▒▒
    ░░░░░░░░░░░░▒▒▒▒░░▒▒▓▓▓▓░░▒▒▒▒▒▒▒▒
  ░░░░▒▒▓▓░░░░░░▒▒▒▒▒▒░░░░░░▒▒▒▒▒▒▒▒▒▒▒▒
  ░░▒▒▒▒▓▓▒▒░░░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  ▒▒░░▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
  ░░░░▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
  ▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
░░░░▒▒▓▓▓▓▓▓████▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
░░▒▒▓▓████████▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
  ▒▒          ▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
              ▒▒▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ░░▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
                ▓▓▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓
                ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
                ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░
                ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ░░▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ░░▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ▒▒▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
              ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░▒▒▒▒▒▒▓▓▓▓▓▓▓▓░░
              ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒░░░░░░░░
            ▒▒▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▒▒▒▒▓▓▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░
            ▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒░░
          ▒▒▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▒▒░░
          ▓▓▓▓▓▓▒▒▒▒▒▒▓▓▒▒▓▓▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓▒▒░░                                                        ░░
        ░░▓▓▓▓▒▒▓▓▒▒▓▓▓▓▒▒▓▓▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓▓░░░░░░░░░░                                      ▒▒██▓▓▓▓▓▓
        ▓▓▓▓▓▓▒▒▓▓▒▒▓▓▓▓▓▓▓▓▒▒▓▓▒▒▓▓▒▒▒▒▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▒▒▓▓▓▓░░░░░░░░░░░░▓▓▒▒░░░░                  ░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓
        ▓▓▓▓▒▒▓▓▓▓▒▒▓▓▓▓▓▓▓▓▒▒▓▓▒▒▓▓▒▒▓▓▒▒▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░    ░░░░░░▒▒▒▒▓▓▓▓▓▓▒▒░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░░░░░  ▒▒▓▓▓▓▓▓▓▓▓▓▓▓▒▒▓▓▒▒
        ▓▓▓▓▒▒▓▓▓▓▓▓▓▓▒▒▓▓▓▓▓▓▒▒▒▒▓▓▒▒▒▒▓▓▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░  ░░░░  ░░▓▓▒▒░░░░░░░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▒▒▓▓██░░
      ░░▓▓▒▒▒▒▓▓▒▒▓▓▓▓▒▒▓▓▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░  ░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▒▒░░░░░░░░░░░░░░▒▒▓▓░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▒▒▓▓▓▓▓▓
      ░░▓▓▒▒▒▒▓▓▒▒▓▓▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  ▓▓▒▒░░░░░░░░░░░░▒▒▓▓▓▓▒▒▒▒░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓
        ░░▒▒▒▒▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓░░▒▒██▓▓▓▓▓▓▓▓▓▓██▓▓████▓▓▓▓▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░░░░░▒▒▓▓▓▓░░░░░░▓▓▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓▓▓▒▒░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░  ░░▓▓▓▓▓▓▓▓▓▓░░░░░░░░▒▒▓▓▓▓░░░░░░▓▓▓▓██▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▒▒▒▒▓▓░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▒▒▒▒░░░░░░░░░░░░▓▓▓▓▓▓▓▓▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒
          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒  ░░░░░░░░  ░░░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒
            ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒░░░░▒▒░░░░░░░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓██▓▓▓▓▓▓▓▓██▒▒
            ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▒▒▒▒▒▒▓▓▓▓░░  ░░░░░░░░░░░░░░░░░░▒▒░░░░░░░░▓▓▓▓▓▓▓▓░░░░░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒
            ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▒▒░░░░▒▒░░░░░░░░░░░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▒▒░░░░░░░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░
              ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▒▒░░░░░░▒▒░░░░░░░░░░░░░░▓▓▓▓▒▒░░░░░░▒▒▓▓████▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒░░
              ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓░░░░░░▓▓▓▓▒▒▒▒▒▒▒▒▒▒░░░░░░░░░░░░░░░░░░▒▒▓▓▓▓▓▓████▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░
                ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒░░░░▓▓▓▓▓▓▒▒▒▒░░░░░░░░░░░░░░░░▒▒██▓▓▓▓▓▓▓▓▓▓██▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▓▓▒▒
                  ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒░░░░░░░░░░░░▒▒▒▒▓▓██▓▓██▓▓▓▓▓▓▓▓▓▓▒▒▓▓██▓▓██▓▓▓▓▒▒░░░░
                    ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░░░░░▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▓▓██
                      ░░▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓  ░░▒▒▓▓▓▓▓▓▒▒▓▓▓▓▓▓▓▓██
                          ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▓▓▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓░░              ░░░░▒▒
                              ▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒
                                  ▒▒▓▓▒▒▒▒▒▒▒▒▒▒▓▓▒▒▓▓▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒
                                        ░░▒▒▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▓▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒▒
                                                      ▒▒▒▒▒▒▒▒▒▒▒▒      ▒▒▒▒▒▒▒▒▒▒▒▒
                                                      ░░▒▒▒▒▒▒▒▒▒▒      ░░▒▒░░▒▒▒▒▒▒
                                                        ░░▒▒▒▒▒▒▒▒        ░░░░▒▒▒▒▒▒
                                                        ░░▒▒▒▒▒▒          ░░░░▒▒▒▒▒▒
                                                          ▒▒▒▒▒▒            ░░▒▒▒▒▒▒
                                                          ▒▒▓▓              ▒▒▒▒▒▒░░
                                                          ▒▒░░              ▒▒▒▒▒▒
                                                        ░░▒▒                ▒▒▒▒
                                                      ░░▒▒                  ▒▒░░
                                        ▒▒▒▒░░░░░░░░▒▒▒▒▒▒▒▒▓▓▓▓          ▒▒▒▒
                                      ▒▒▒▒▒▒▒▒▒▒░░▒▒▒▒▒▒▒▒▒▒░░            ░░▒▒░░
                                          ░░░░▒▒▒▒▒▒░░▒▒                ▒▒▒▒▒▒▒▒▒▒██▒▒
                                        ▒▒▒▒▒▒░░  ░░▒▒░░      ░░▒▒░░▒▒▒▒░░▒▒▒▒▒▒░░
                                      ▓▓▓▓░░      ░░▒▒░░  ░░▒▒▒▒▒▒▒▒░░▒▒▒▒░░▒▒▒▒
                                      ░░          ▒▒▓▓    ▓▓    ░░▒▒▒▒░░░░▒▒░░▒▒
                                                  ░░▒▒        ▒▒▒▒          ▒▒▒▒░░
                                                  ░░        ██▒▒              ▒▒▒▒
                                                            ▒▒

     */
}
