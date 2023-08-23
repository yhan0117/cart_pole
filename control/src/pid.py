def pid(z,K,zr):
    # two controller stages
    # match
        # case swing up
            # Lyapunov based

        # case near top
            # calculate error z - r
            # calculate feedback u = -K*e
            # saturation
    
    u = -np.sum(np.multiply(z-zr,K))
    # return u
    return u