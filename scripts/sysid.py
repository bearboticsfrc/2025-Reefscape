MOTORS = []
COUNT = 5

def main() -> None:
    for i in range(1, COUNT):
        print(f"TalonFX-{i}:")
        Ks = float(input("\tKs: "))
        Kv = float(input("\tKv: "))
        Ka = float(input("\tKa: "))
        Kp = float(input("\tKp: "))
        Kd = float(input("\tKd: "))

        MOTORS.append((Ks, Kv, Ka, Kp, Kd))

    sum_Ks = sum([g[0] for g in MOTORS])
    sum_Kv = sum([g[1] for g in MOTORS])
    sum_Ka = sum([g[2] for g in MOTORS])
    sum_Kp = sum([g[3] for g in MOTORS])
    sum_Kd = sum([g[4] for g in MOTORS])

    print(f"Ks = {sum_Ks / COUNT}\nKv = {sum_Kv / COUNT}\nKa = {sum_Ka / COUNT}\nKp = {sum_Kp / COUNT}\nKd = {sum_Kd / COUNT}")

if __name__ == "__main__":
    main()