__author__ = "Leonid Voldman"
__copyright__ = "Copyright 2024"
__credits__ = ["VoldmanTech"]
__license__ = "SLA"
__version__ = "3.6.0"
__maintainer__ = "Leonid Voldman"
__email__ = "vleonid@voldman.com"
__status__ = "Tool"


import sqlite3
import time, datetime
from queue import Queue 


from bs1_utils import print_log, print_inf, print_err, print_DEBUG, exptTrace, s16, s32, num2binstr, set_parm, get_parm, globalEventQ, smartLocker



class StatisticSQLite:
    
    def __init__(self, _devName:str,  _db_name:str, _tbl_name:str, _refresh_timeout:int = 10):

        self.__db_name = _db_name
        self.__tbl_name = _tbl_name
        self.__refresh_timeout = _refresh_timeout
        self.__sqliteConnection = None
        self.__cursor = None
        self.__success_counter = 0
        self.devName = _devName
        self.devNotificationQ = Queue()                 # for compatability


        self.__sqliteConnection = sqlite3.connect(self.__db_name)
        self.__cursor = self.__sqliteConnection.cursor()

        query = 'select sqlite_version();'
        self.__cursor.execute(query)
        result = self.__cursor.fetchall()
        print_log(f'SQLite Version is {result}')

        if self.__sqliteConnection:
            self.__sqliteConnection.close()


    def StartDB(self)->bool:
        try:
            self.__sqliteConnection = sqlite3.connect(self.__db_name)
            self.__cursor = self.__sqliteConnection.cursor()


            # Create a table
            self.__cursor.execute(f'''
            CREATE TABLE IF NOT EXISTS {self.__tbl_name} (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                date TEXT NOT NULL,
                hour INT NOT NULL,
                minute INT NOT NULL,
                second INT NOT NULL,
                result BLOB NOT NULL,
                success_cnt INT NOT NULL,        
                fail_cnt INT NOT NULL       
            )
            ''')

            self.__cursor.execute(f'''SELECT success_cnt FROM {self.__tbl_name} ORDER BY id DESC''') 
            _last_rec = self.__cursor.fetchone()
            if _last_rec is not None:
                self.__success_counter = _last_rec[0]

        except sqlite3.Error as error:
            exptTrace(error)
            print_err(f'Error accessing DB - {error}')
            self.__clear()
            if self.__sqliteConnection:
                self.__sqliteConnection.close()
                print_log('SQLite Connection closed')
            return False

        except Exception as ex:
            exptTrace(ex)
            print_err(f'Error accessing DB: {ex}')
            self.__clear()
            if self.__sqliteConnection:
                self.__sqliteConnection.close()
                print_log('SQLite Connection closed')
            return False
            
        if self.__sqliteConnection:
            self.__sqliteConnection.close()
        
        return True


    def  mDev_stop(self)->bool:                 # for compatability
        return True

    def __clear(self):

        if self.__sqliteConnection:
            self.__sqliteConnection.close()
            print_log('SQLite Connection closed')
    
    def __del__(self):
        self.__clear()    


    def recordDB(self, _res:bool = True)->bool:
        if _res is None:
            _res = True
        try:

            self.__sqliteConnection = sqlite3.connect(self.__db_name)
            self.__cursor = self.__sqliteConnection.cursor()


            _sc_cnt:int = -1
            _fl_cnt:int = -1
            self.__cursor.execute(f'''SELECT success_cnt, fail_cnt FROM {self.__tbl_name} ORDER BY id DESC''') 
            _rec = self.__cursor.fetchone() 

            if _rec is None:
                _sc_cnt = 0
                _fl_cnt = 0

            else:
                if _rec[0] == '':
                    _sc_cnt = 0
                else:
                    _sc_cnt = int(_rec[0])
                if _rec[1] == '':
                    _fl_cnt = 0
                else:
                    _fl_cnt = int(_rec[1])


            if _res:
                _sc_cnt += 1
            else:
                _fl_cnt += 1

            print_log(f'Adding statistic {_res}, succes rcr = {_rec} = {_sc_cnt}/{_fl_cnt}')

            _date:str = datetime.datetime.now().strftime(f"%Y_%m_%d")
            _hour:int = int(datetime.datetime.now().strftime(f"%H"))
            _min:int = int(datetime.datetime.now().strftime(f"%M"))
            _sec:int = int(datetime.datetime.now().strftime(f"%S"))
            self.__cursor.execute(f'INSERT INTO {self.__tbl_name} (date, hour, minute, second, result, success_cnt, fail_cnt) VALUES (?, ?, ?, ?, ?, ?, ?)', \
                        (_date, _hour, _min, _sec, _res, _sc_cnt, _fl_cnt))
        
            self.__sqliteConnection.commit()
            self.__success_counter = _sc_cnt

        except sqlite3.Error as error:
            exptTrace(error)
            print_err(f'Error accessing DB - {error}')
            if self.__sqliteConnection:
                self.__sqliteConnection.close()

            return False

        except Exception as ex:
            exptTrace(ex)
            print_err(f'Error accessing DB: {ex}')
            if self.__sqliteConnection:
                self.__sqliteConnection.close()
            return False


        if self.__sqliteConnection:
            self.__sqliteConnection.close()
        return True


    def resetCounters(self)->bool:
        try:
            self.__sqliteConnection = sqlite3.connect(self.__db_name)
            self.__cursor = self.__sqliteConnection.cursor()

            self.__cursor.execute(f'''SELECT id FROM {self.__tbl_name} ORDER BY id DESC''') 
            _last_rec = self.__cursor.fetchone()
            if _last_rec is None:
                print_log(f'DB is empty')
                return True
            self.__cursor.execute(f'''UPDATE {self.__tbl_name} set success_cnt = 0, fail_cnt =0 where id={_last_rec[0]}''') 
            self.__sqliteConnection.commit()
            self.__success_counter = 0
            print_log(f'Reseting counters. Last record = {_last_rec[0]}')

        except sqlite3.Error as error:
            exptTrace(error)
            print_err(f'Error reseting counters - {error}')
            if self.__sqliteConnection:
                self.__sqliteConnection.close()
            return False

        except Exception as ex:
            exptTrace(ex)
            print_err(f'Error reseting counters : {ex}')
            if self.__sqliteConnection:
                self.__sqliteConnection.close()
            return False

        if self.__sqliteConnection:
            self.__sqliteConnection.close()

        return True
        

    @property
    def cursor(self):
        return self.__cursor
    
    @property
    def success_counter(self)->int:
        return self.__success_counter
    
    def set_parms(self, parms)->bool:
        pass

#------------------------   UNITEST SECTION -----------------
if __name__ == "__main__":

    def add_record(_conn, _cursor, _res:bool):
        _sc_cnt:int = -1
        _fl_cnt:int = -1
        print(f'Adding {_res}')
        _cursor.execute('''SELECT * FROM production ORDER BY id DESC''') 
        _rec = _cursor.fetchone() 

        if _rec is None:
            _sc_cnt = 0
            _fl_cnt = 0

        else:
            if _rec[5] == '':
                _sc_cnt = 0
            else:
                _sc_cnt = int(_rec[5])
            if _rec[6] == '':
                _fl_cnt = 0
            else:
                 _fl_cnt = int(_rec[6])


        if _res:
            _sc_cnt += 1
        else:
            _fl_cnt += 1

        print(f'succes rcr = {_rec} = {_sc_cnt}/{_fl_cnt}')

        _date:str = datetime.datetime.now().strftime(f"%Y_%m_%d")
        _hour:int = int(datetime.datetime.now().strftime(f"%H"))
        _min:int = int(datetime.datetime.now().strftime(f"%M"))
        _sec:int = int(datetime.datetime.now().strftime(f"%S"))
        _cursor.execute('INSERT INTO production (date, hour, minute, second, result, success_cnt, fail_cnt) VALUES (?, ?, ?, ?, ?, ?, ?)', \
                       (_date, _hour, _min, _sec, _res, _sc_cnt, _fl_cnt))
    
        _conn.commit()
        
                # Query the data
        _cursor.execute('SELECT * FROM production')
        rows = _cursor.fetchall()

        # Print the results
        for row in rows:
            print(row)
        print('-----------------------------')

    def test1():
        try:
            # Connect to SQLite database (or create it if it doesn't exist)
            sqliteConnection = sqlite3.connect('statistic.db')

            # Create a cursor object to interact with the database
            cursor = sqliteConnection.cursor()

            query = 'select sqlite_version();'
            cursor.execute(query)
            result = cursor.fetchall()
            print(f'SQLite Version is {result}')


            # Create a table
            cursor.execute('''
            CREATE TABLE IF NOT EXISTS production (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                date TEXT NOT NULL,
                hour INT NOT NULL,
                minute INT NOT NULL,
                second INT NOT NULL,
                result BLOB NOT NULL,
                success_cnt INT NOT NULL,        
                fail_cnt INT NOT NULL       
            )
            ''')


            # Query the data
            cursor.execute('SELECT * FROM production')
            rows = cursor.fetchall()

            # Print the results
            for row in rows:
                print(row)

            add_record(sqliteConnection, cursor, False)
            add_record(sqliteConnection, cursor, False)
            add_record(sqliteConnection, cursor, True)
            add_record(sqliteConnection, cursor, False)
            add_record(sqliteConnection, cursor, True)
            add_record(sqliteConnection, cursor, False)





        except sqlite3.Error as error:
            print('Error occurred - ', error)

        except Exception as ex:
            print(f'Exception : {ex}')
            
        finally:
        
            if sqliteConnection:
                sqliteConnection.close()
                print('SQLite Connection closed')
    
    def print_tbl(_cursor, __tbl):
        _cursor.execute(f'SELECT * FROM {__tbl}')
        rows = _cursor.fetchall()

        # Print the results
        for row in rows:
            print(row)
        print('-----------------------------')   


    __db = 'test.db'
    __tbl = 'statistic'
    t1:StatisticSQLite = StatisticSQLite('DB', __db, __tbl)
    t1.StartDB()
    # Query the data
    _cursor = t1.cursor
    
    t1.recordDB(True)
    print_tbl(_cursor, __tbl)

    t1.recordDB(True)
    print_tbl(_cursor, __tbl)

    t1.recordDB(False)
    print_tbl(_cursor, __tbl)

    t1.resetCounters()
    print_tbl(_cursor, __tbl)

    t1.recordDB(False)
    print_tbl(_cursor, __tbl)

    t1.recordDB(True)
    print_tbl(_cursor, __tbl)

    t1.recordDB(False)
    print_tbl(_cursor, __tbl)


     
