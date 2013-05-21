Header                                          header    # a copy of the header of the image from
                                                          # which the features were extracted
infrastructure_camera_tracker/TrackedFeature[]  features  # features in the image
bool                                            new_set   # is this message the first (representing
                                                          # a new set) or is it an update?