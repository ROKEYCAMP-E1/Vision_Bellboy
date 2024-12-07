import sqlite3


connection = sqlite3.connect('system_monitor.db')

cursor=connection.cursor()

def create_tables():
    create_info_table='''
    CREATE TABLE IF NOT EXISTS info_talble(
    No_ INTEGER PRIMARY KEY AUTOINCREMENT,
    datetime TEXT,
    type TEXT,
    DETECT_ID INTEGER,
    info TEXT
    );
'''

    create_month_data_table='''
    CREATE TABLE IF NOT EXISTS month_data_table(
    datetime TEXT PRIMARY KEY,
    day_user INTEGER,
    detect_per_user FLOAT,
    avg_time FLOAT
    );
'''

    create_user_table='''
    CREATE TABLE IF NOT EXISTS user_table(
    id TEXT PRIMARY KEY,
    password TEXT

    );
'''

    cursor.execute(create_info_table)
    cursor.execute(create_month_data_table)
    cursor.execute(create_user_table)

    # delete_all_entries_query = "DELETE FROM info_table;"
    # cursor.execute(delete_all_entries_query)

    # delete_all_entries_query = "DELETE FROM month_data_table;"
    # cursor.execute(delete_all_entries_query)

    # delete_all_entries_query = "DELETE FROM create_user_table;"
    # cursor.execute(delete_all_entries_query)

    # print("Tables created and emptied successfully.")

def main():
    create_tables()


if __name__ == "__main__":
    main()

    connection.commit()
    connection.close()