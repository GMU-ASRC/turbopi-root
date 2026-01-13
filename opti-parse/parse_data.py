import io, re

import pandas as pd

def cleanup_data_csv(csv_data: str) -> pd.DataFrame:
    df = pd.read_csv(io.StringIO(csv_data))
    # NOTE: The captured data provides a lot of extra detail that won't be necessary
    # when parsing this info such as the position (x,y,z) of all the markers, etc.
    # Using the markers, the opti-tracking software creates a rigidbody with position
    # (x,y,z) and rotation (quaternion). This script will analyze the turbopi's
    # capabilities using that rigidbody.

    # NOTE: Skipping the first two rows because they contain the frame and time values.
    re_pat: str = r"^Rigid Body(\.(\d+))?$"
    columns = df.columns[2:]
    cols_str = "\n".join(columns)
    matches = re.finditer(re_pat, cols_str, re.MULTILINE)
    cols_to_save = [match.group() for match in matches]
    cols_to_remove = [x for x in columns if x not in cols_to_save]
    for col in cols_to_remove:
        del df[col]

    # NOTE: The 'Name' and 'ID' row are removed because they aren't needed
    df = df.drop(index=[0, 1]).reset_index(drop=True)

    # NOTE: Flatten 'Quantity' and 'Dimension' rows into a single column. Then,
    # remove the two rows, combine them, and rename the columns
    quantities: pd.Series = df.iloc[0].values[2:]
    dimensions: pd.Series = df.iloc[1].values[2:]
    new_names: list[str] = ["Frame", "Time (seconds)"]
    new_names.extend([
        f"{q}.{d}" for q, d in zip(quantities, dimensions)
    ])

    df = df.drop(index=[0, 1]).reset_index(drop=True)
    df.rename(columns={
        df.columns[i]: new_names[i]
        for i in range(len(df.columns))
    }, inplace=True)
    return df

def setup_csv(csv_path: str, print_info: bool = True) -> pd.DataFrame:
    with open(csv_path, "r") as f:
        content = f.read()

    # NOTE: First line provides extra information about the captured data
    # TODO: parse this information and turn it into something readable in `stdout`.
    first_line_end = content.find("\n")
    info = content[:first_line_end].strip()
    if print_info:
        values: list[str] = info.split(',')
        print("Info about data captured from opti-track camera:")
        # TODO: convert to dict, if needed
        for i in range(0, len(values), 2):
            k, v = values[i], values[i+1]
            print(f"{k:>25}: {v}")

    data = content[first_line_end:].strip()
    return cleanup_data_csv(data)


actual_csv_path: str = "optitrack-data/Take 2025-12-05 03.07.49 PM turbopi-01.csv"
sample_csv_path: str = "sample.csv"

df = setup_csv(sample_csv_path)
with open("test.csv", "w") as f:
    print(df, file=f)
