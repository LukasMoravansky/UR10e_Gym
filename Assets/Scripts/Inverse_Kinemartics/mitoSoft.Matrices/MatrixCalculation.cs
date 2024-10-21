using System;

namespace mitoSoft.Matrices
{
    public static class MatrixCalculation
    {

        public static Matrix NormalizeColumns(Matrix matrix)
        {
            var result = new Matrix(matrix.RowCount, matrix.ColumnCount);

            for (int col = 0; col < matrix.ColumnCount; col++)
            {
                var sum = matrix.GetColumnSum(col);
                for (int row = 0; row < matrix.RowCount; row++)
                {
                    result[row, col] = matrix[row, col] / sum;
                }
            }

            return result;
        }

        public static Matrix NormalizedRows(Matrix matrix)
        {
            var result = new Matrix(matrix.RowCount, matrix.ColumnCount);

            for (int row = 0; row < matrix.ColumnCount; row++)
            {
                var sum = matrix.GetRowSum(row);
                for (int col = 0; col < matrix.ColumnCount; col++)
                {
                    result[row, col] = matrix[row, col] / sum;
                }
            }

            return result;
        }

        public static Matrix Multiplicate(Matrix matrix1, Matrix matrix2)
        {
            Matrix result = new Matrix(matrix1.RowCount, matrix2.ColumnCount);
            //all rows
            for (int row = 0; row < matrix1.RowCount; row++)
            {
                //all columns
                for (int col = 0; col < matrix2.ColumnCount; col++)
                {
                    decimal dbl = 0;
                    //Scalarproduct
                    for (int k = 0; k < matrix1.ColumnCount; k++)
                    {
                        dbl += matrix1[row, k] * matrix2[k, col];
                    }
                    result[row, col] = dbl;
                }
            }

            return result;
        }

        public static Matrix Transpose(Matrix matrix)
        {
            var result = new Matrix(matrix.ColumnCount, matrix.RowCount);
            //all rows
            for (int row = 0; row < matrix.RowCount; row++)
            {
                //all columns
                for (int col = 0; col < matrix.ColumnCount; col++)
                {
                    result[col, row] = matrix[row, col];
                }
            }

            return result;
        }

        public static Matrix Subtract(Matrix matrix1, Matrix matrix2)
        {
            return CalculateArithmetic(matrix1, matrix2, (v1, v2) => v1 - v2);
        }

        public static Matrix Add(this Matrix matrix1, Matrix matrix2)
        {
            return CalculateArithmetic(matrix1, matrix2, (v1, v2) => v1 + v2);
        }

        private delegate decimal ArithmeticFunction(decimal value1, decimal value2);

        private static Matrix CalculateArithmetic(Matrix matrix1, Matrix matrix2, ArithmeticFunction arithmetic)
        {            
            if (matrix2.RowCount != matrix1.RowCount || matrix2.ColumnCount != matrix1.ColumnCount)
            {
                throw new InvalidOperationException("Invalid matrix size.");
            }

            var result = new Matrix(matrix1.RowCount, matrix1.ColumnCount);

            //all rows
            for (int row = 0; row < matrix2.RowCount; row++)
            {
                //all columns
                for (int col = 0; col < matrix1.ColumnCount; col++)
                {
                    result[row, col] = arithmetic(matrix1[row, col], matrix2[row, col]);
                }
            }

            return result;
        }
    }
}