algo = "bug0"


if algo == "bug0" or algo == "bug1" or algo == "bug2":
    print("RUNNING {}...".format(algo))
    with open(algo + ".py", "r") as rnf:
        exec(rnf.read())
else:
    print("UNKNOWN ALGORITHM ENTERD!")
