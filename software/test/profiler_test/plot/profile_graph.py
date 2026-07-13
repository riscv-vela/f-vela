import re
import matplotlib.pyplot as plt
import random

# Expected log format: type, start_time, end_time
# Example: 0, 100, 180
pattern = re.compile(r'^\s*\d+\s*,\s*\d+\s*,\s*\d+\s*$')


# file open
def read_log(src):
    result_list = []

    with open(src, "r") as f:
        for line in f:
            line = line.strip()
            if pattern.match(line):
                nums = [int(x.strip()) for x in line.split(",")]
                result_list.append(nums)

    return result_list


# random color
def generate_random_color():
    # Generate a random color in hex format
    random_color = "#{:06x}".format(random.randint(0, 0xFFFFFE))
    return random_color


# visualize
def visualize(records):
    if len(records) == 0:
        print("No valid log lines found.")
        return

    visual_data = {}

    for i in range(len(records)):
        op_type = records[i][0]
        start_time = records[i][1]
        end_time = records[i][2]

        visual_data[i] = {
            'Type': op_type,
            'Latency': (start_time, end_time - start_time),
            'Color': generate_random_color()
        }

    starts = [visual_data[tag]['Latency'][0] for tag in visual_data]
    ends = [
        visual_data[tag]['Latency'][0] + visual_data[tag]['Latency'][1]
        for tag in visual_data
    ]

    fig, ax = plt.subplots(1, 1, figsize=(12, 4))

    ax.axvspan(min(starts), max(ends), facecolor='lightgrey', alpha=0.3)

    for tag in visual_data:
        if visual_data[tag]['Type'] == 0:
            ax.broken_barh(
                [visual_data[tag]['Latency']],
                (20, 8),
                facecolors=visual_data[tag]['Color'],
                edgecolor='none'
            )
            # ax.text(visual_data[tag]['Latency'][0] + visual_data[tag]['Latency'][1] / 2,
            #         24, tag, ha='center', va='center', color='black', fontsize=10)

        elif visual_data[tag]['Type'] == 1:
            ax.broken_barh(
                [visual_data[tag]['Latency']],
                (10, 8),
                facecolors=visual_data[tag]['Color'],
                edgecolor='none'
            )
            # ax.text(visual_data[tag]['Latency'][0] + visual_data[tag]['Latency'][1] / 2,
            #         14, tag, ha='center', va='center', color='black', fontsize=10)

        elif visual_data[tag]['Type'] == 2:
            ax.broken_barh(
                [visual_data[tag]['Latency']],
                (0, 8),
                facecolors=visual_data[tag]['Color'],
                edgecolor=visual_data[tag]['Color']
            )
            # ax.text(visual_data[tag]['Latency'][0] + visual_data[tag]['Latency'][1] / 2,
            #         4, tag, ha='center', va='center', color='black', fontsize=10)

    total_time = max(ends) - min(starts)
    margin = total_time / 20 if total_time > 0 else 1

    ax.set_xlim(min(starts) - margin, max(ends) + margin)
    ax.set_xlabel("Time")
    ax.set_yticks([24, 14, 4])
    ax.set_yticklabels(["ld", "ex", "st"])
    ax.tick_params(axis='y', which='both', length=0)
    ax.set_title("Gemmini")

    plt.tight_layout()

    print("Overall Start Time:", min(starts))
    print("Overall End Time:", max(ends))
    print("Duration:", total_time)
